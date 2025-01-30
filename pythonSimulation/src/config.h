#ifndef INC_CONFIG_H
#define INC_CONFIG_H

#define USE_TRAFO_33V 0  // 0: high voltage grid. 1: test with 33V trafo (~30.5V with 9 extra windings for 15 cell LFP battery)

#if USE_TRAFO_33V == 1
#define VGRID_TRATIO 7  // transformer winding ratio
#elif USE_TRAFO_33V == 0
#define VGRID_TRATIO 1  // no transformer
#endif

#define VGRID_AMP 325/VGRID_TRATIO

#endif /* INC_CONFIG_H */
