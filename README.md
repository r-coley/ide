ID/Space.c
==========
int  atadebug=0;              /* 0=off; 9=full */

int  ata_intr_mode=0;     /* 1 = Interrupt,  0 = Polling */

int  atapi_intr_mode=0;   /* 1 = Interrupt,  0 = Polling */


A standard UnixWare machine wont have the RegisterIRQ()

`
enum intr_trigger {
        INTR_TRIGGER_INVALID    = -1,
        INTR_TRIGGER_CONFORM    = 0,
        INTR_TRIGGER_EDGE       = 1,
        INTR_TRIGGER_LEVEL      = 2
};

/*
 * Dynamically register an interrupt handler
 *
 * RegisterIRQ(11, &ataintr, SPL6, INTR_TRIGGER_LEVEL);
 */
int
RegisterIRQ(int irq,int (*func)(),int pri,enum intr_trigger le)
{
        extern int      intnull(), (*ivect[])(), nintr;
        extern uchar_t  intpri[];
        int     old_intr_mask;

        if (ivect[irq] == intnull) {
                ivect[irq]=func;
                intpri[irq]=pri;
                if ((int)(irq+1) > nintr)
                        nintr=irq+1;
                picinit();
        }
        else
        if (ivect[irq] == func)
                /*printf("Warning: IRQ %d already installed\n",irq);*/ ;
        else
                printf("Error: IRQ %d has handler %08x - check config\n",
                        irq,ivect[irq]);

        old_intr_mask=level_intr_mask;
        switch(le) {
        case INTR_TRIGGER_EDGE:
                /*printf("Setting IRQ%d to EDGE\n",irq);*/
                level_intr_mask &= ~ELCR_MASK(irq);
                break;
        case INTR_TRIGGER_LEVEL:
                /*printf("Setting IRQ%d to Level\n",irq);*/
                level_intr_mask |= ELCR_MASK(irq);
                break;
        }
        if (old_intr_mask != level_intr_mask)
                elcr_write_trigger(irq, le);

        /*elcr_show();*/
        return -1;
}`

