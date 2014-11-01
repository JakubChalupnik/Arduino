#ifndef _LED_SEG_H
#define _LED_SEG_H

#define _A_SEG 3
#define _B_SEG 6
#define _C_SEG 1
#define _D_SEG 7
#define _E_SEG 0
#define _F_SEG 5
#define _G_SEG 2
#define _P_SEG 4

#define seg_mac(a,b,c,d,e,f,g,p) (((a << _A_SEG) | (b << _B_SEG) | (c << _C_SEG) | (d << _D_SEG) | (e << _E_SEG) | (f << _F_SEG) | (g << _G_SEG) | (p << _P_SEG)))

//----------------------------------------------------------------------------
// 7seg definitions

unsigned char seg_hex_table[] = {
    seg_mac(1, 1, 1, 1, 1, 1, 0, 0),             // 0
    seg_mac(0, 1, 1, 0, 0, 0, 0, 0),             // 1
    seg_mac(1, 1, 0, 1, 1, 0, 1, 0),             // 2
    seg_mac(1, 1, 1, 1, 0, 0, 1, 0),             // 3
    seg_mac(0, 1, 1, 0, 0, 1, 1, 0),             // 4
    seg_mac(1, 0, 1, 1, 0, 1, 1, 0),             // 5
    seg_mac(1, 0, 1, 1, 1, 1, 1, 0),             // 6
    seg_mac(1, 1, 1, 0, 0, 0, 0, 0),             // 7
    seg_mac(1, 1, 1, 1, 1, 1, 1, 0),             // 8
    seg_mac(1, 1, 1, 1, 0, 1, 1, 0),             // 9
    seg_mac(1, 1, 1, 0, 1, 1, 1, 0),             // A
    seg_mac(0, 0, 1, 1, 1, 1, 1, 0),             // B
    seg_mac(1, 0, 0, 1, 1, 1, 0, 0),             // C
    seg_mac(0, 1, 1, 1, 1, 0, 1, 0),             // D
    seg_mac(1, 0, 0, 1, 1, 1, 1, 0),             // E
    seg_mac(1, 0, 0, 0, 1, 1, 1, 0),             // F
};

#endif // _LED_SEG_H
