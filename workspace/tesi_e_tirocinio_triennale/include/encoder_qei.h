#ifndef ENCODER_QEI_H
#define ENCODER_QEI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Inizializza il timer GPT320 in Phase Counting Mode per l'encoder (Pin D6/D7)
    void setupGPT_QEI(void);

    // Restituisce i tick accumulati dall'Encoder (Pin D6/D7 -> P106/P107) a 32 bit con segno
    int32_t getEncoderTicks(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_QEI_H