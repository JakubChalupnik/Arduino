#define _A_SEG 0
#define _B_SEG 1
#define _C_SEG 2
#define _D_SEG 3
#define _E_SEG 4
#define _F_SEG 5
#define _G_SEG 6
#define _P_SEG 7

#define SegMac(a,b,c,d,e,f,g,p) (((a << _A_SEG) | (b << _B_SEG) | (c << _C_SEG) | (d << _D_SEG) | (e << _E_SEG) | (f << _F_SEG) | (g << _G_SEG) | (p << _P_SEG)))

//----------------------------------------------------------------------------
// 7seg definitions

unsigned char SegHexTable [] = {
    SegMac(1, 1, 1, 1, 1, 1, 0, 0),             // 0
    SegMac(0, 1, 1, 0, 0, 0, 0, 0),             // 1
    SegMac(1, 1, 0, 1, 1, 0, 1, 0),             // 2
    SegMac(1, 1, 1, 1, 0, 0, 1, 0),             // 3
    SegMac(0, 1, 1, 0, 0, 1, 1, 0),             // 4
    SegMac(1, 0, 1, 1, 0, 1, 1, 0),             // 5
    SegMac(1, 0, 1, 1, 1, 1, 1, 0),             // 6
    SegMac(1, 1, 1, 0, 0, 0, 0, 0),             // 7
    SegMac(1, 1, 1, 1, 1, 1, 1, 0),             // 8
    SegMac(1, 1, 1, 1, 0, 1, 1, 0),             // 9
    SegMac(1, 1, 1, 0, 1, 1, 1, 0),             // A
    SegMac(0, 0, 1, 1, 1, 1, 1, 0),             // B
    SegMac(1, 0, 0, 1, 1, 1, 0, 0),             // C
    SegMac(0, 1, 1, 1, 1, 0, 1, 0),             // D
    SegMac(1, 0, 0, 1, 1, 1, 1, 0),             // E
    SegMac(1, 0, 0, 0, 1, 1, 1, 0),             // F
    SegMac(0, 0, 0, 0, 0, 0, 0, 0),             // Blank
    SegMac(1, 1, 0, 0, 0, 1, 1, 0),             // Degree
    SegMac(0, 0, 0, 0, 0, 0, 1, 0),             // Dash
};

