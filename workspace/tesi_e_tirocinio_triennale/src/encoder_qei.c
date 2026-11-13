#include "encoder_qei.h"
#include <bsp_api.h>

void setupGPT_QEI(void)
{
    // Sblocco protezione registri PFS
    R_PMISC->PWPR = 0b00000000;
    R_PMISC->PWPR = 0b01000000;

    // ENCODER: Pin D6 (P106 -> GTIOC0A) e D7 (P107 -> GTIOC0B)
    R_PFS->PORT[1].PIN[6].PmnPFS_b.PSEL = 0b00011; // Funzione periferica GPT
    R_PFS->PORT[1].PIN[6].PmnPFS_b.PMR = 1;        // Modalità periferica

    R_PFS->PORT[1].PIN[7].PmnPFS_b.PSEL = 0b00011;
    R_PFS->PORT[1].PIN[7].PmnPFS_b.PMR = 1;

    // Ripristino protezione registri I/O
    R_PMISC->PWPR = 0b00000000;
    R_PMISC->PWPR = 0b10000000;

    // Attivazione clock periferica GPT320 (bit 5)
    R_MSTP->MSTPCRD &= ~(1U << 5);

    // Configurazione timer GPT320 in Phase Counting Mode 4
    R_GPT0->GTCR = 0x00000000;
    R_GPT0->GTUPSR = 0x00006900;
    R_GPT0->GTDNSR = 0x00009600;
    R_GPT0->GTPR = 0xFFFFFFFF;
    R_GPT0->GTCNT = 0x00000000;

    // Avvio contatore
    R_GPT0->GTCR_b.CST = 1;
}

int32_t getEncoderTicks(void)
{
    return (int32_t)R_GPT0->GTCNT;
}