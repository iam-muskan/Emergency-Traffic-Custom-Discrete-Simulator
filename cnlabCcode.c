#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define MAX_NODES 20
#define MAX_EVENTS 200000
#define MAX_PKTS_PER_QUEUE 100
#define MAX_PKT_POOL 200000

#define SIM_TIME 20.0
#define AREA_SIZE 1000.0

#define TX_RANGE 250.0
#define CS_RANGE 550.0
#define INT_RANGE 350.0

#define CW_MIN 32
#define CW_MAX 1024

#define SLOT_TIME 0.000020
#define SIFS 0.000010
#define DIFS (2 * SIFS)
#define PROP_DELAY 0.000001

#define DATA_BITS 8000.0
#define ACK_BITS 112.0
#define DATA_TX_TIME (DATA_BITS / 2e6)
#define ACK_TX_TIME (ACK_BITS / 2e6)
#define ACK_TIMEOUT (DIFS + ACK_TX_TIME + 4 * PROP_DELAY)

#define MAX_RETRIES 7

typedef enum {
    EV_MOB = 0,
    EV_PKT_ARR,
    EV_BACKOFF,
    EV_TX_END,
    EV_ACK_TX,
    EV_ACK_RX,
    EV_ACK_TO,
    EV_LOG,
    EV_ADAPT_CW
} event_type_t;

typedef struct {
    double time;
    event_type_t type;
    int node;
    int pkt_id;
    int dst;
} event_t;

typedef struct {
    int id;
    int src;
    int dst;
    int prio;              /* 0 = best effort, 1 = emergency */
    double arrival_time;
    int retries;
} packet_t;

typedef struct {
    double x, y;

    packet_t queue[MAX_PKTS_PER_QUEUE];
    int q_head, q_tail, q_len;

    int cw;
    int backoff;
    int tx_busy;

    double tx_start;
    double tx_end;

    double total_delay;
    double total_bits;

    int tx_success;
    int tx_total;

    int pending_acks[MAX_PKT_POOL];
    int recent_collisions;

    double meas_start;
    int is_adaptive_mode;
} node_t;

/* ---------------- Globals ---------------- */
static node_t nodes[MAX_NODES];
static event_t heap[MAX_EVENTS];
static packet_t pkt_pool[MAX_PKT_POOL];

static int heap_size = 0;
static int pkt_id_gen = 0;
static double sim_time = 0.0;

static FILE *csv_file = NULL;

/* ---------------- Heap ---------------- */
void heap_push(event_t e) {
    if (heap_size >= MAX_EVENTS) return;

    heap[heap_size] = e;
    int i = heap_size++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (heap[p].time <= heap[i].time) break;
        event_t t = heap[p];
        heap[p] = heap[i];
        heap[i] = t;
        i = p;
    }
}

event_t heap_pop(void) {
    event_t res = heap[0];
    heap[0] = heap[--heap_size];

    int i = 0;
    while (1) {
        int l = 2 * i + 1;
        int r = 2 * i + 2;
        int smallest = i;

        if (l < heap_size && heap[l].time < heap[smallest].time) smallest = l;
        if (r < heap_size && heap[r].time < heap[smallest].time) smallest = r;
        if (smallest == i) break;

        event_t t = heap[i];
        heap[i] = heap[smallest];
        heap[smallest] = t;
        i = smallest;
    }

    return res;
}

/* ---------------- Utility ---------------- */
double dist_nodes(int i, int j) {
    double dx = nodes[i].x - nodes[j].x;
    double dy = nodes[i].y - nodes[j].y;
    return sqrt(dx * dx + dy * dy);
}

packet_t *new_packet(int src, int dst) {
    if (pkt_id_gen >= MAX_PKT_POOL) return NULL;

    packet_t *p = &pkt_pool[pkt_id_gen];
    p->id = pkt_id_gen++;
    p->src = src;
    p->dst = dst;
    p->prio = rand() % 2;      /* 0 or 1 */
    p->arrival_time = sim_time;
    p->retries = 0;
    return p;
}

/* ---------------- Queue ---------------- */
void enqueue(node_t *n, packet_t *p) {
    if (n->q_len >= MAX_PKTS_PER_QUEUE) return;

    n->queue[n->q_tail] = *p;
    n->q_tail = (n->q_tail + 1) % MAX_PKTS_PER_QUEUE;
    n->q_len++;
}

int peek_head(node_t *n, packet_t *p) {
    if (n->q_len == 0) return 0;
    *p = n->queue[n->q_head];
    return 1;
}

void dequeue_head(node_t *n) {
    if (n->q_len == 0) return;
    n->q_head = (n->q_head + 1) % MAX_PKTS_PER_QUEUE;
    n->q_len--;
}

/* ---------------- PHY / Channel ---------------- */
int channel_busy(int nid) {
    for (int i = 0; i < MAX_NODES; i++) {
        if (i == nid) continue;

        if (dist_nodes(nid, i) < CS_RANGE &&
            nodes[i].tx_busy &&
            sim_time >= nodes[i].tx_start &&
            sim_time <= nodes[i].tx_end) {
            return 1;
        }
    }
    return 0;
}

int collision(int tx, int rx, double tstart, double tend) {
    for (int i = 0; i < MAX_NODES; i++) {
        if (i == tx) continue;

        if (dist_nodes(i, rx) < INT_RANGE && nodes[i].tx_busy) {
            double istart = nodes[i].tx_start;
            double iend   = nodes[i].tx_end;

            if (fmax(tstart, istart) < fmin(tend, iend)) {
                return 1;
            }
        }
    }
    return 0;
}

/* ---------------- Adaptive CW ---------------- */
void apply_adaptive_cw(int nid) {
    node_t *n = &nodes[nid];

    if (!n->is_adaptive_mode) return;

    double window = sim_time - n->meas_start;
    if (window < 3.0) return;

    packet_t head;
    int has_head = peek_head(n, &head);

    /* Priority boost for emergency packets */
    if (has_head && head.prio == 1) {
        n->cw = (int)fmax((double)(CW_MIN / 4), n->cw * 0.5);
        n->meas_start = sim_time;
        n->recent_collisions = 0;
        return;
    }

    double recent_delay = (n->tx_success > 0) ? (n->total_delay / n->tx_success) : 0.0;
    double target_delay = 0.001;
    double collision_rate = (n->tx_total > 0) ? ((double)n->recent_collisions / n->tx_total) : 0.0;

    double delay_factor = recent_delay / target_delay;
    double collision_factor = collision_rate;

    /* Prototype adaptive formula */
    double adapt_factor = 1.0 + 0.2 * collision_factor - 0.1 * delay_factor;

    if (adapt_factor < 0.5) adapt_factor = 0.5;
    if (adapt_factor > 1.5) adapt_factor = 1.5;

    n->cw = (int)fmax((double)CW_MIN, fmin((double)CW_MAX, n->cw * adapt_factor));

    n->meas_start = sim_time;
    n->recent_collisions = 0;
}

/* ---------------- Events ---------------- */
void ev_mobility(int nid) {
    nodes[nid].x = ((double)rand() / RAND_MAX) * AREA_SIZE;
    nodes[nid].y = ((double)rand() / RAND_MAX) * AREA_SIZE;

    heap_push((event_t){sim_time + 5.0 + (rand() % 5), EV_MOB, nid, 0, 0});
}

void ev_pkt_arrival(int nid) {
    /* Higher traffic so graphs are not flat */
    double lambda = 40.0;
    double u = (double)rand() / RAND_MAX;
    if (u >= 1.0) u = 0.999999;
    double interarr = -log(1.0 - u) / lambda;

    int dst = rand() % MAX_NODES;
    while (dst == nid) dst = rand() % MAX_NODES;

    packet_t *p = new_packet(nid, dst);
    if (p != NULL) {
        node_t *n = &nodes[nid];
        int was_empty = (n->q_len == 0);
        enqueue(n, p);

        /* Start contention when first packet enters queue */
        if (was_empty && n->q_len > 0) {
            n->backoff = rand() % n->cw;
            heap_push((event_t){sim_time + DIFS, EV_BACKOFF, nid, 0, 0});
        }
    }

    if (sim_time + interarr < SIM_TIME) {
        heap_push((event_t){sim_time + interarr, EV_PKT_ARR, nid, 0, 0});
    }
}

void ev_backoff(int nid) {
    node_t *n = &nodes[nid];

    if (n->q_len == 0) return;

    if (channel_busy(nid)) {
        /* Freeze and retry shortly */
        heap_push((event_t){sim_time + SLOT_TIME, EV_BACKOFF, nid, 0, 0});
        return;
    }

    packet_t head;
    if (!peek_head(n, &head)) return;

    /* Proposed mode: priority CW hint before contention */
    if (n->is_adaptive_mode && head.prio == 1) {
        if (n->cw > CW_MIN / 4) {
            n->cw = (int)fmax((double)(CW_MIN / 4), n->cw * 0.5);
        }
    }

    if (n->backoff > 0) {
        n->backoff--;
        heap_push((event_t){sim_time + SLOT_TIME, EV_BACKOFF, nid, 0, 0});
        return;
    }

    /* Start TX */
    n->tx_busy = 1;
    n->tx_start = sim_time;
    n->tx_end = sim_time + DATA_TX_TIME;
    n->pending_acks[head.id] = 1;
    n->tx_total++;   /* Important for PDR */

    heap_push((event_t){sim_time + DATA_TX_TIME, EV_TX_END, nid, head.id, head.dst});
    heap_push((event_t){sim_time + ACK_TIMEOUT, EV_ACK_TO, nid, head.id, head.dst});
}

void ev_tx_end(int nid, int pktid, int dst) {
    node_t *n = &nodes[nid];
    n->tx_busy = 0;

    /* Prototype-friendly: not every overlap becomes failure */
    if (collision(nid, dst, n->tx_start, n->tx_end) && (rand() % 100 < 70)) {
        n->recent_collisions++;
        n->pending_acks[pktid] = 0;

        if (pktid >= 0 && pktid < pkt_id_gen) {
            pkt_pool[pktid].retries++;
            if (pkt_pool[pktid].retries > MAX_RETRIES) {
                dequeue_head(n);
                n->cw = CW_MIN;
                if (n->q_len > 0) {
                    n->backoff = rand() % n->cw;
                    heap_push((event_t){sim_time + DIFS, EV_BACKOFF, nid, 0, 0});
                }
                return;
            }
        }

        n->cw = (int)fmin((double)CW_MAX, n->cw * 2.0);
        n->backoff = rand() % n->cw;
        heap_push((event_t){sim_time + DIFS, EV_BACKOFF, nid, 0, 0});
        return;
    }

    /* Success path → receiver sends ACK */
    heap_push((event_t){sim_time + SIFS, EV_ACK_TX, dst, pktid, nid});
}

void ev_ack_tx(int nid, int pktid, int orig_tx) {
    (void)nid;
    heap_push((event_t){sim_time + ACK_TX_TIME, EV_ACK_RX, orig_tx, pktid, nid});
}

void ev_ack_rx(int txid, int pktid, int rxid) {
    (void)rxid;

    node_t *n = &nodes[txid];
    n->pending_acks[pktid] = 0;

    double pkt_delay = sim_time - pkt_pool[pktid].arrival_time;
    n->total_delay += pkt_delay;
    n->total_bits += DATA_BITS;
    n->tx_success++;

    dequeue_head(n);

    /* BEB success */
    n->cw = (int)fmax((double)CW_MIN, n->cw / 2.0);

    if (n->q_len > 0) {
        n->backoff = rand() % n->cw;
        heap_push((event_t){sim_time + DIFS, EV_BACKOFF, txid, 0, 0});
    }
}

void ev_ack_timeout(int txid, int pktid, int dst) {
    (void)dst;

    node_t *n = &nodes[txid];
    if (!n->pending_acks[pktid]) return;

    n->pending_acks[pktid] = 0;
    n->recent_collisions++;

    if (pktid >= 0 && pktid < pkt_id_gen) {
        pkt_pool[pktid].retries++;
        if (pkt_pool[pktid].retries > MAX_RETRIES) {
            dequeue_head(n);
            n->cw = CW_MIN;
            if (n->q_len > 0) {
                n->backoff = rand() % n->cw;
                heap_push((event_t){sim_time + DIFS, EV_BACKOFF, txid, 0, 0});
            }
            return;
        }
    }

    n->cw = (int)fmin((double)CW_MAX, n->cw * 2.0);
    n->backoff = rand() % n->cw;
    heap_push((event_t){sim_time + DIFS, EV_BACKOFF, txid, 0, 0});
}

void ev_adapt_cw(int nid) {
    apply_adaptive_cw(nid);
    heap_push((event_t){sim_time + 2.0, EV_ADAPT_CW, nid, 0, 0});
}

void ev_log(void) {
    double sum_delay = 0.0;
    double sum_thru = 0.0;
    double sum_thru_sq = 0.0;
    double pdr = 0.0;

    for (int i = 0; i < MAX_NODES; i++) {
        node_t *n = &nodes[i];

        double avg_delay = (n->tx_success > 0) ? (n->total_delay / n->tx_success) : 0.0;
        double throughput = (sim_time > 0.0) ? (n->total_bits / sim_time) : 0.0;
        double node_pdr = (n->tx_total > 0) ? ((double)n->tx_success / n->tx_total) : 0.0;

        sum_delay += avg_delay;
        sum_thru += throughput;
        sum_thru_sq += throughput * throughput;
        pdr += node_pdr;
    }

    double jain_fairness = 0.0;
    if (sum_thru_sq > 0.0) {
        jain_fairness = (sum_thru * sum_thru) / (MAX_NODES * sum_thru_sq);
    }

    fprintf(csv_file, "%.1f,%.6f,%.3f,%.1f,%.3f\n",
            sim_time,
            sum_delay / MAX_NODES,
            pdr / MAX_NODES,
            sum_thru / MAX_NODES / 1000.0,
            jain_fairness);

    /* Log every 1 second for smoother plots */
    heap_push((event_t){sim_time + 1.0, EV_LOG, 0, 0, 0});
}

/* ---------------- Run simulation ---------------- */
void run_sim(int adaptive_mode) {
    char fname[32];
    sprintf(fname, adaptive_mode ? "proposed.csv" : "baseline.csv");

    csv_file = fopen(fname, "w");
    if (!csv_file) {
        printf("Error opening %s\n", fname);
        return;
    }

    fprintf(csv_file, "time,avg_delay,pdr,throughput_kbps,fairness\n");

    memset(nodes, 0, sizeof(nodes));
    memset(pkt_pool, 0, sizeof(pkt_pool));
    heap_size = 0;
    pkt_id_gen = 0;
    sim_time = 0.0;

    for (int i = 0; i < MAX_NODES; i++) {
        nodes[i].x = ((double)rand() / RAND_MAX) * AREA_SIZE;
        nodes[i].y = ((double)rand() / RAND_MAX) * AREA_SIZE;
        nodes[i].cw = CW_MIN;
        nodes[i].backoff = rand() % CW_MIN;
        nodes[i].is_adaptive_mode = adaptive_mode;
        nodes[i].meas_start = 0.0;

        heap_push((event_t){i * 0.02, EV_MOB, i, 0, 0});
        heap_push((event_t){i * 0.05, EV_PKT_ARR, i, 0, 0});

        if (adaptive_mode) {
            heap_push((event_t){1.0 + i * 0.1, EV_ADAPT_CW, i, 0, 0});
        }
    }

    heap_push((event_t){1.0, EV_LOG, 0, 0, 0});

    while (heap_size > 0 && sim_time < SIM_TIME) {
        event_t ev = heap_pop();
        sim_time = ev.time;

        switch (ev.type) {
            case EV_MOB:      ev_mobility(ev.node); break;
            case EV_PKT_ARR:  ev_pkt_arrival(ev.node); break;
            case EV_BACKOFF:  ev_backoff(ev.node); break;
            case EV_TX_END:   ev_tx_end(ev.node, ev.pkt_id, ev.dst); break;
            case EV_ACK_TX:   ev_ack_tx(ev.node, ev.pkt_id, ev.dst); break;
            case EV_ACK_RX:   ev_ack_rx(ev.node, ev.pkt_id, ev.dst); break;
            case EV_ACK_TO:   ev_ack_timeout(ev.node, ev.pkt_id, ev.dst); break;
            case EV_LOG:      ev_log(); break;
            case EV_ADAPT_CW: ev_adapt_cw(ev.node); break;
            default: break;
        }
    }

    fclose(csv_file);
}

/* ---------------- Main ---------------- */
int main(void) {
    srand(12345);

    printf("QoS-Adaptive IEEE 802.11 MANET Simulator\n");

    printf("[1/2] Pure IEEE 802.11 DCF (BEB only)...\n");
    run_sim(0);

    printf("[2/2] Proposed QoS-Adaptive (Priority + Delay/Collision)...\n");
    run_sim(1);

    printf("COMPLETE: baseline.csv + proposed.csv\n");
    printf("Use MATLAB plots.m\n");

    return 0;
}
