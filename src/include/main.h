
typedef struct {
    int initSpeed;
    int targetSpeed;
    int adjFreq;
    int maxAcc;
    uint32_t runtime_ms;
    uint32_t firstStep;
} adjustParams;

typedef struct
{
    /* data */
    uint32_t lastStep;
    uint32_t runTime;
} adjustSum;
