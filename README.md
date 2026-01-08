1| ID/Space.c
2| ==========
3| int  atadebug=0;              /* 0=off; 9=full */
4| 
5| int  ata_intr_mode=0;     /* 1 = Interrupt,  0 = Polling */
6| 
7| int  atapi_intr_mode=0;   /* 1 = Interrupt,  0 = Polling */
8| 
9| 
10| A standard UnixWare machine wont have the RegisterIRQ()
11| 
12| <br>
13| ```
13| enum intr_trigger {
14|         INTR_TRIGGER_INVALID    = -1,
15|         INTR_TRIGGER_CONFORM    = 0,
16|         INTR_TRIGGER_EDGE       = 1,
17|         INTR_TRIGGER_LEVEL      = 2
18| };
19| 
20| /*
21|  * Dynamically register an interrupt handler
22|  *
23|  * RegisterIRQ(11, &ataintr, SPL6, INTR_TRIGGER_LEVEL);
24|  */
25| int
26| RegisterIRQ(int irq,int (*func)(),int pri,enum intr_trigger le)
27| {
28|         extern int      intnull(), (*ivect[])(), nintr;
29|         extern uchar_t  intpri[];
30|         int     old_intr_mask;
31| 
32|         if (ivect[irq] == intnull) {
33|                 ivect[irq]=func;
34|                 intpri[irq]=pri;
35|                 if ((int)(irq+1) > nintr)
36|                         nintr=irq+1;
37|                 picinit();
38|         }
39|         else
40|         if (ivect[irq] == func)
41|                 /*printf("Warning: IRQ %d already installed\n",irq);*/ ;
42|         else
43|                 printf("Error: IRQ %d has handler %08x - check config\n",
44|                         irq,ivect[irq]);
45| 
46|         old_intr_mask=level_intr_mask;
47|         switch(le) {
48|         case INTR_TRIGGER_EDGE:
49|                 /*printf("Setting IRQ%d to EDGE\n",irq);*/
50|                 level_intr_mask &= ~ELCR_MASK(irq);
51|                 break;
52|         case INTR_TRIGGER_LEVEL:
53|                 /*printf("Setting IRQ%d to Level\n",irq);*/
54|                 level_intr_mask |= ELCR_MASK(irq);
55|                 break;
56|         }
57|         if (old_intr_mask != level_intr_mask)
58|                 elcr_write_trigger(irq, le);
59| 
60|         /*elcr_show();*/
61|         return -1;
62| }
63| 
64| ```