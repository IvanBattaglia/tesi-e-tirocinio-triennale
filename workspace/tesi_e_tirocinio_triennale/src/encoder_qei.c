#include "encoder_qei.h"
        #include <bsp_api.h>

        void setupGPT_QEI(void)
        {
            // =========================================================================
            // STEP 1: SBLOCCO E ROUTING DEI PIN HARDWARE (PmnPFS)
            // =========================================================================
            // Per modificare i registri PFS occorre sbloccare la protezione PWPR:
            // 1. Scrivere 0x00(h)=0b00000000(b) su PWPR per azzerare il bit B0WI (PFSWE Write Enable)
            // 2. Scrivere 0x40(h)=0b01000000(b) su PWPR per impostare PFSWE a 1 (abilita la scrittura sui registri PFS)
            R_PMISC->PWPR = 0b00000000;
            R_PMISC->PWPR = 0b01000000;

            // --- ENCODER 0: Pin D6 (P106 -> GTIOC0A) e D7 (P107 -> GTIOC0B) ---
            // P106 (Porta 1, Pin 6) -> Canale A di GPT320
            R_PFS->PORT[1].PIN[6].PmnPFS_b.PSEL = 0b00011; // PSEL = 0x03 (Funzione periferica GPT)
            R_PFS->PORT[1].PIN[6].PmnPFS_b.PMR = 1;        // PMR = 1 (Dirige il pin da GPIO a Periferica)

            // P107 (Porta 1, Pin 7) -> Canale B di GPT320
            R_PFS->PORT[1].PIN[7].PmnPFS_b.PSEL = 0b00011; // PSEL = 0x03
            R_PFS->PORT[1].PIN[7].PmnPFS_b.PMR = 1;        // PMR = 1

            // --- ENCODER 1: Pin D2 (P105 -> GTIOC1A) e D3 (P104 -> GTIOC1B) ---
            // P105 (Porta 1, Pin 5) -> Canale A di GPT321
            R_PFS->PORT[1].PIN[5].PmnPFS_b.PSEL = 0b00011; // PSEL = 0x03
            R_PFS->PORT[1].PIN[5].PmnPFS_b.PMR = 1;        // PMR = 1

            // P104 (Porta 1, Pin 4) -> Canale B di GPT321
            R_PFS->PORT[1].PIN[4].PmnPFS_b.PSEL = 0b00011; // PSEL = 0x03
            R_PFS->PORT[1].PIN[4].PmnPFS_b.PMR = 1;        // PMR = 1

            // Ripristino della protezione dei registri I/O (PWPR)
            R_PMISC->PWPR = 0b00000000; // Azzera PFSWE
            R_PMISC->PWPR = 0b10000000; // Ripristina B0WI (Blocco globale)

            // =========================================================================
            // STEP 2: ALIMENTAZIONE CLOCK PERIFERICA GPT (MSTPCRD)
            // =========================================================================
            // Il registro MSTPCRD gestisce il risparmio energetico delle periferiche GPT.
            // Azzerando il bit 5 (MSTPD5) si cancella lo stato di Module Stop per i timer GPT a 32-bit (GPT320 e GPT321).
            R_MSTP->MSTPCRD &= ~(1U << 5); // 1U= il numero "1" formattato come unsigned int (32 bit); <<5 = sposta il bit 1 a sinistra di 5 posizioni (bit 5 = 1 e tutti gli altri 0 in questo caso); ~ = NOT bitwise, inverte tutti i bit (ora tutti i bit tranne il bit 5 sono "1"); &= = AND bitwise,mantiene solo i bit che sono 1 in entrambi gli operandi (cioè tutti i bit tranne il bit 5 rimangono invariati, mentre il bit 5 diventa 0) => ho attivato l'alimentazione dei timer GPT320 e GPT321 (MSTPD5=0)

            // =========================================================================
            // STEP 3: CONFIGURAZIONE TIMER GPT320 (Encoder 0 - Canale 0)
            // =========================================================================
            R_GPT0->GTCR = 0x00000000;   // MD[2:0] = 0b000 (Saw-wave mode) e Timer ferito (CST = 0)
            R_GPT0->GTUPSR = 0x00006900; // Sorgenti UP-Count per Phase Counting Mode 4 (Table 22.14)
            R_GPT0->GTDNSR = 0x00009600; // Sorgenti DOWN-Count per Phase Counting Mode 4 (Table 22.14)
            R_GPT0->GTPR = 0xFFFFFFFF;   // Fondo scala contatore a 32 bit (4.294.967.295)
            R_GPT0->GTCNT = 0x00000000;  // Azzeramento contatore iniziale

            // =========================================================================
            // STEP 4 : CONFIGURAZIONE TIMER GPT321(Encoder 1 - Canale 1)
            // =========================================================================
            R_GPT1->GTCR = 0x00000000;   // MD[2:0] = 0b000 (Saw-wave mode) e Timer fermo (CST = 0)
            R_GPT1->GTUPSR = 0x00006900; // Sorgenti UP-Count per Phase Counting Mode 4 (Table 22.14)
            R_GPT1->GTDNSR = 0x00009600; // Sorgenti DOWN-Count per Phase Counting Mode 4 (Table 22.14)
            R_GPT1->GTPR = 0xFFFFFFFF;   // Fondo scala contatore a 32 bit
            R_GPT1->GTCNT = 0x00000000;  // Azzeramento contatore iniziale

            // =========================================================================
            // STEP 5: AVVIO CONTEMPORANEO DEI CONTATORI HARDWARE
            // =========================================================================
            R_GPT0->GTCR_b.CST = 1; // Avvia il contatore GPT320
            R_GPT1->GTCR_b.CST = 1; // Avvia il contatore GPT321
        }

        int32_t getEncoder0Ticks(void)
        {
            // Lettura del registro di conteggio fisico a 32-bit.
            // Il cast esplicito a (int32_t) trasforma il valore senza segno in valore relativo con segno (+/-).
            return (int32_t)R_GPT0->GTCNT;
        }

        int32_t getEncoder1Ticks(void)
        {
            return (int32_t)R_GPT1->GTCNT;
        }