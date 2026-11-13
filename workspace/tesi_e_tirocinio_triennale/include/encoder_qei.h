#ifndef ENCODER_QEI_H
#define ENCODER_QEI_H

#include <stdint.h>

#ifdef __cplusplus // Se il compilatore è C++, allora includi il blocco di codice seguente
extern "C"
{
#endif

    // Inizializza i timer GPT320 e GPT321 del microcontrollore Renesas RA4M1 in Phase Counting Mode 4 (risoluzione 4x) per la lettura hardware di 2 encoder.
    void setupGPT_QEI(void);

    // Restituisce i tick accumulati dall'Encoder 0 (GPT320 - Pin D6/D7 -> P106/P107) Conteggio a 32 bit con segno (int32_t)
    int32_t getEncoder0Ticks(void);

    // Restituisce i tick accumulati dall'Encoder 1 (GPT321 - Pin D2/D3 -> P105/P104) Conteggio a 32 bit con segno (int32_t)
    int32_t getEncoder1Ticks(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_QEI_H