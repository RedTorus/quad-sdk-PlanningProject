/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) eval_jac_g_leg_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[40] = {36, 1, 0, 36, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
static const casadi_int casadi_s1[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s2[200] = {28, 36, 0, 1, 2, 3, 8, 13, 16, 18, 20, 22, 26, 32, 38, 45, 52, 60, 67, 74, 82, 89, 96, 104, 111, 118, 126, 127, 128, 129, 134, 139, 142, 143, 144, 145, 149, 155, 161, 0, 1, 2, 3, 4, 5, 10, 11, 3, 4, 5, 9, 11, 5, 9, 10, 0, 6, 1, 7, 2, 8, 3, 9, 10, 11, 3, 4, 5, 9, 10, 11, 3, 4, 5, 9, 10, 11, 0, 6, 9, 10, 11, 12, 13, 1, 7, 9, 10, 11, 14, 15, 2, 8, 9, 10, 12, 13, 14, 15, 0, 6, 9, 10, 11, 16, 17, 1, 7, 9, 10, 11, 18, 19, 2, 8, 9, 10, 16, 17, 18, 19, 0, 6, 9, 10, 11, 20, 21, 1, 7, 9, 10, 11, 22, 23, 2, 8, 9, 10, 20, 21, 22, 23, 0, 6, 9, 10, 11, 24, 25, 1, 7, 9, 10, 11, 26, 27, 2, 8, 9, 10, 24, 25, 26, 27, 0, 1, 2, 3, 4, 5, 10, 11, 3, 4, 5, 9, 11, 5, 9, 10, 6, 7, 8, 3, 9, 10, 11, 3, 4, 5, 9, 10, 11, 3, 4, 5, 9, 10, 11};

/* eval_jac_g_leg:(w[36],p[13])->(jac_g[28x36,161nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95;
  a0=-1.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  a1=5.0000000000000000e-001;
  a2=arg[0]? arg[0][3] : 0;
  a3=arg[0]? arg[0][27] : 0;
  a2=(a2+a3);
  a3=2.;
  a2=(a2/a3);
  a4=sin(a2);
  a5=arg[0]? arg[0][11] : 0;
  a6=arg[0]? arg[0][35] : 0;
  a7=(a5+a6);
  a8=(a7/a3);
  a9=arg[0]? arg[0][4] : 0;
  a10=arg[0]? arg[0][28] : 0;
  a9=(a9+a10);
  a9=(a9/a3);
  a10=sin(a9);
  a11=cos(a9);
  a12=(a10/a11);
  a13=(a8*a12);
  a13=(a4*a13);
  a14=cos(a2);
  a15=arg[0]? arg[0][10] : 0;
  a16=arg[0]? arg[0][34] : 0;
  a17=(a15+a16);
  a18=(a17/a3);
  a19=(a10/a11);
  a20=(a18*a19);
  a20=(a14*a20);
  a13=(a13-a20);
  a13=(a1*a13);
  a20=arg[1]? arg[1][0] : 0;
  a21=(1./a20);
  a22=(a13-a21);
  if (res[0]!=0) res[0][3]=a22;
  a22=(a14*a8);
  a23=(a4*a18);
  a22=(a22+a23);
  a22=(a1*a22);
  if (res[0]!=0) res[0][4]=a22;
  a23=(a8/a11);
  a23=(a4*a23);
  a24=(a18/a11);
  a24=(a14*a24);
  a23=(a23-a24);
  a23=(a1*a23);
  if (res[0]!=0) res[0][5]=a23;
  a24=-6.1099999999999994e-005;
  a25=(a18*a8);
  a25=(a24*a25);
  a26=2.7500000000000001e-005;
  a27=arg[0]? arg[0][9] : 0;
  a28=arg[0]? arg[0][33] : 0;
  a29=(a27+a28);
  a30=(a29/a3);
  a31=(a30*a8);
  a31=(a26*a31);
  a25=(a25+a31);
  a31=-2.5702007500000001e-001;
  a32=(a30*a18);
  a32=(a31*a32);
  a25=(a25+a32);
  a32=-3.6600000000000002e-005;
  a33=casadi_sq(a18);
  a34=(a32*a33);
  a25=(a25+a34);
  a34=3.6600000000000002e-005;
  a35=casadi_sq(a30);
  a36=(a34*a35);
  a25=(a25+a36);
  a36=3.1492299499999998e-001;
  a6=(a6-a5);
  a5=(a6/a20);
  a37=(a36*a5);
  a25=(a25-a37);
  a16=(a16-a15);
  a15=(a16/a20);
  a37=(a26*a15);
  a25=(a25+a37);
  a37=6.1099999999999994e-005;
  a28=(a28-a27);
  a27=(a28/a20);
  a38=(a37*a27);
  a25=(a25+a38);
  a25=(a14*a25);
  a38=(a18*a8);
  a38=(a32*a38);
  a39=-2.8197203500000001e-001;
  a40=(a30*a8);
  a40=(a39*a40);
  a38=(a38+a40);
  a40=(a30*a18);
  a40=(a26*a40);
  a38=(a38+a40);
  a40=casadi_sq(a8);
  a41=(a24*a40);
  a38=(a38+a41);
  a41=(a37*a35);
  a38=(a38+a41);
  a41=-2.7500000000000001e-005;
  a5=(a41*a5);
  a38=(a38+a5);
  a5=2.8997103499999999e-001;
  a15=(a5*a15);
  a38=(a38+a15);
  a27=(a32*a27);
  a38=(a38+a27);
  a38=(a4*a38);
  a25=(a25-a38);
  a25=(a1*a25);
  if (res[0]!=0) res[0][6]=a25;
  a38=(a18*a8);
  a27=(a38*a11);
  a27=(a32*a27);
  a15=(a30*a8);
  a42=(a15*a11);
  a42=(a39*a42);
  a27=(a27+a42);
  a42=(a30*a18);
  a43=(a42*a11);
  a43=(a26*a43);
  a27=(a27+a43);
  a43=(a24*a40);
  a44=(a11*a43);
  a27=(a27+a44);
  a44=(a37*a35);
  a45=(a11*a44);
  a27=(a27+a45);
  a6=(a6/a20);
  a45=(a26*a6);
  a46=(a11*a45);
  a27=(a27-a46);
  a16=(a16/a20);
  a46=(a5*a16);
  a47=(a11*a46);
  a27=(a27+a47);
  a28=(a28/a20);
  a47=(a34*a28);
  a48=(a11*a47);
  a27=(a27-a48);
  a14=(a14*a27);
  a27=(a18*a8);
  a48=(a37*a11);
  a49=(a27*a48);
  a50=(a30*a8);
  a51=(a41*a11);
  a52=(a50*a51);
  a49=(a49+a52);
  a52=(a30*a18);
  a53=2.5702007500000001e-001;
  a54=(a53*a11);
  a55=(a52*a54);
  a49=(a49+a55);
  a55=(a34*a33);
  a56=(a11*a55);
  a49=(a49+a56);
  a35=(a32*a35);
  a56=(a11*a35);
  a49=(a49+a56);
  a56=(a36*a6);
  a57=(a11*a56);
  a49=(a49+a57);
  a57=(a26*a16);
  a58=(a11*a57);
  a49=(a49-a58);
  a58=(a37*a28);
  a59=(a11*a58);
  a49=(a49-a59);
  a4=(a4*a49);
  a14=(a14-a4);
  a14=(a1*a14);
  if (res[0]!=0) res[0][7]=a14;
  a4=cos(a9);
  a49=sin(a2);
  a59=(a18*a49);
  a60=(a59/a11);
  a2=cos(a2);
  a61=(a8*a2);
  a62=(a61/a11);
  a60=(a60+a62);
  a60=(a4*a60);
  a9=sin(a9);
  a62=(a30*a11);
  a61=(a61*a10);
  a62=(a62+a61);
  a59=(a59*a10);
  a62=(a62+a59);
  a62=(a62/a11);
  a62=(a62/a11);
  a59=(a30/a11);
  a62=(a62-a59);
  a62=(a9*a62);
  a60=(a60+a62);
  a60=(a1*a60);
  a60=(-a60);
  if (res[0]!=0) res[0][8]=a60;
  a62=(1./a20);
  a59=(-a62);
  if (res[0]!=0) res[0][9]=a59;
  a59=(a8*a2);
  a61=(a18*a49);
  a59=(a59+a61);
  a59=(a59/a11);
  a59=(a59/a11);
  a59=(a9*a59);
  a59=(a1*a59);
  a59=(-a59);
  if (res[0]!=0) res[0][10]=a59;
  a61=arg[1]? arg[1][11] : 0;
  a63=arg[0]? arg[0][21] : 0;
  a64=(a61*a63);
  a65=arg[1]? arg[1][10] : 0;
  a66=arg[0]? arg[0][22] : 0;
  a67=(a65*a66);
  a64=(a64-a67);
  a67=arg[1]? arg[1][7] : 0;
  a68=arg[0]? arg[0][19] : 0;
  a69=(a67*a68);
  a64=(a64-a69);
  a69=arg[1]? arg[1][8] : 0;
  a70=arg[0]? arg[0][18] : 0;
  a71=(a69*a70);
  a64=(a64+a71);
  a71=arg[1]? arg[1][4] : 0;
  a72=arg[0]? arg[0][16] : 0;
  a73=(a71*a72);
  a64=(a64-a73);
  a73=arg[1]? arg[1][5] : 0;
  a74=arg[0]? arg[0][15] : 0;
  a75=(a73*a74);
  a64=(a64+a75);
  a75=arg[1]? arg[1][2] : 0;
  a76=arg[0]? arg[0][12] : 0;
  a77=(a75*a76);
  a64=(a64+a77);
  a77=arg[1]? arg[1][1] : 0;
  a78=arg[0]? arg[0][13] : 0;
  a79=(a77*a78);
  a64=(a64-a79);
  a64=(a4*a64);
  a79=arg[0]? arg[0][5] : 0;
  a80=arg[0]? arg[0][29] : 0;
  a79=(a79+a80);
  a79=(a79/a3);
  a3=cos(a79);
  a80=(a61*a3);
  a81=sin(a79);
  a82=(a65*a81);
  a80=(a80-a82);
  a82=arg[0]? arg[0][23] : 0;
  a83=(a80*a82);
  a84=arg[1]? arg[1][12] : 0;
  a85=(a3*a66);
  a85=(a84*a85);
  a83=(a83-a85);
  a85=(a81*a63);
  a85=(a84*a85);
  a83=(a83+a85);
  a85=(a69*a3);
  a86=(a67*a81);
  a85=(a85-a86);
  a86=arg[0]? arg[0][20] : 0;
  a87=(a85*a86);
  a83=(a83+a87);
  a87=arg[1]? arg[1][9] : 0;
  a88=(a3*a68);
  a88=(a87*a88);
  a83=(a83-a88);
  a88=(a81*a70);
  a88=(a87*a88);
  a83=(a83+a88);
  a88=(a73*a3);
  a89=(a71*a81);
  a88=(a88-a89);
  a89=arg[0]? arg[0][17] : 0;
  a90=(a88*a89);
  a83=(a83+a90);
  a90=arg[1]? arg[1][6] : 0;
  a91=(a3*a72);
  a91=(a90*a91);
  a83=(a83-a91);
  a91=(a81*a74);
  a91=(a90*a91);
  a83=(a83+a91);
  a91=(a75*a3);
  a92=(a77*a81);
  a91=(a91-a92);
  a92=arg[0]? arg[0][14] : 0;
  a93=(a91*a92);
  a83=(a83+a93);
  a93=arg[1]? arg[1][3] : 0;
  a94=(a81*a76);
  a94=(a93*a94);
  a83=(a83+a94);
  a94=(a3*a78);
  a94=(a93*a94);
  a83=(a83-a94);
  a83=(a9*a83);
  a64=(a64-a83);
  a64=(a1*a64);
  if (res[0]!=0) res[0][11]=a64;
  a83=-2.4951959999999999e-002;
  a94=(a18*a8);
  a94=(a83*a94);
  a95=(a30*a8);
  a95=(a32*a95);
  a94=(a94+a95);
  a95=(a30*a18);
  a95=(a37*a95);
  a94=(a94+a95);
  a40=(a41*a40);
  a94=(a94+a40);
  a33=(a26*a33);
  a94=(a94+a33);
  a6=(a37*a6);
  a94=(a94+a6);
  a16=(a34*a16);
  a94=(a94+a16);
  a16=-3.2950960000000001e-002;
  a28=(a16*a28);
  a94=(a94+a28);
  a4=(a4*a94);
  a94=(a32*a49);
  a38=(a38*a94);
  a28=(a39*a49);
  a15=(a15*a28);
  a38=(a38+a15);
  a15=(a26*a49);
  a42=(a42*a15);
  a38=(a38+a42);
  a27=(a27*a2);
  a27=(a37*a27);
  a38=(a38+a27);
  a50=(a50*a2);
  a50=(a41*a50);
  a38=(a38+a50);
  a52=(a52*a2);
  a53=(a53*a52);
  a38=(a38+a53);
  a43=(a49*a43);
  a38=(a38+a43);
  a44=(a49*a44);
  a38=(a38+a44);
  a55=(a2*a55);
  a38=(a38+a55);
  a35=(a2*a35);
  a38=(a38+a35);
  a45=(a49*a45);
  a38=(a38-a45);
  a56=(a2*a56);
  a38=(a38+a56);
  a46=(a49*a46);
  a38=(a38+a46);
  a57=(a2*a57);
  a38=(a38-a57);
  a47=(a49*a47);
  a38=(a38-a47);
  a58=(a2*a58);
  a38=(a38-a58);
  a9=(a9*a38);
  a4=(a4-a9);
  a4=(a1*a4);
  if (res[0]!=0) res[0][12]=a4;
  a9=(1./a20);
  a38=(-a9);
  if (res[0]!=0) res[0][13]=a38;
  a38=cos(a79);
  a58=(a84*a11);
  a47=(a58*a63);
  a57=(a11*a82);
  a46=(a65*a57);
  a47=(a47-a46);
  a46=(a11*a86);
  a56=(a67*a46);
  a47=(a47-a56);
  a56=(a87*a11);
  a45=(a56*a70);
  a47=(a47+a45);
  a45=(a11*a89);
  a35=(a71*a45);
  a47=(a47-a35);
  a35=(a90*a11);
  a55=(a35*a74);
  a47=(a47+a55);
  a55=(a11*a92);
  a44=(a77*a55);
  a47=(a47-a44);
  a44=(a93*a11);
  a43=(a44*a76);
  a47=(a47+a43);
  a47=(a38*a47);
  a79=sin(a79);
  a57=(a61*a57);
  a43=(a84*a11);
  a53=(a43*a66);
  a57=(a57-a53);
  a46=(a69*a46);
  a57=(a57+a46);
  a46=(a87*a11);
  a53=(a46*a68);
  a57=(a57-a53);
  a45=(a73*a45);
  a57=(a57+a45);
  a45=(a90*a11);
  a53=(a45*a72);
  a57=(a57-a53);
  a55=(a75*a55);
  a57=(a57+a55);
  a55=(a93*a11);
  a53=(a55*a78);
  a57=(a57-a53);
  a57=(a79*a57);
  a47=(a47-a57);
  a47=(a1*a47);
  if (res[0]!=0) res[0][14]=a47;
  a66=(a84*a66);
  a57=(a61*a82);
  a66=(a66-a57);
  a57=(a69*a86);
  a66=(a66-a57);
  a68=(a87*a68);
  a66=(a66+a68);
  a68=(a73*a89);
  a66=(a66-a68);
  a72=(a90*a72);
  a66=(a66+a72);
  a78=(a93*a78);
  a66=(a66+a78);
  a78=(a75*a92);
  a66=(a66-a78);
  a38=(a38*a66);
  a63=(a84*a63);
  a82=(a65*a82);
  a63=(a63-a82);
  a86=(a67*a86);
  a63=(a63-a86);
  a70=(a87*a70);
  a63=(a63+a70);
  a89=(a71*a89);
  a63=(a63-a89);
  a74=(a90*a74);
  a63=(a63+a74);
  a76=(a93*a76);
  a63=(a63+a76);
  a92=(a77*a92);
  a63=(a63-a92);
  a79=(a79*a63);
  a38=(a38-a79);
  a38=(a1*a38);
  if (res[0]!=0) res[0][15]=a38;
  a79=(-a20);
  if (res[0]!=0) res[0][16]=a79;
  if (res[0]!=0) res[0][17]=a0;
  a79=(-a20);
  if (res[0]!=0) res[0][18]=a79;
  if (res[0]!=0) res[0][19]=a0;
  a79=(-a20);
  if (res[0]!=0) res[0][20]=a79;
  if (res[0]!=0) res[0][21]=a0;
  a79=-5.0000000000000000e-001;
  if (res[0]!=0) res[0][22]=a79;
  a63=(a34*a8);
  a92=(a37*a18);
  a63=(a63-a92);
  a63=(a1*a63);
  a92=3.2950960000000001e-002;
  a92=(a92/a20);
  a76=(a63-a92);
  if (res[0]!=0) res[0][23]=a76;
  a76=(a26*a49);
  a74=(a8*a76);
  a31=(a31*a49);
  a89=(a18*a31);
  a74=(a74+a89);
  a39=(a39*a2);
  a89=(a8*a39);
  a74=(a74+a89);
  a89=(a26*a2);
  a70=(a18*a89);
  a74=(a74+a70);
  a70=(a34*a49);
  a86=(a37*a2);
  a70=(a70+a86);
  a70=(a29*a70);
  a74=(a74+a70);
  a74=(a1*a74);
  a70=(a32*a2);
  a86=(a37*a49);
  a70=(a70+a86);
  a70=(a70/a20);
  a86=(a74-a70);
  if (res[0]!=0) res[0][24]=a86;
  a28=(a11*a28);
  a86=(a8*a28);
  a15=(a11*a15);
  a82=(a18*a15);
  a86=(a86+a82);
  a51=(a2*a51);
  a82=(a8*a51);
  a86=(a86+a82);
  a54=(a2*a54);
  a82=(a18*a54);
  a86=(a86+a82);
  a82=(a32*a10);
  a66=(a8*a82);
  a86=(a86+a66);
  a66=(a37*a10);
  a78=(a18*a66);
  a86=(a86+a78);
  a78=(a11*a49);
  a78=(a37*a78);
  a72=(a2*a11);
  a72=(a32*a72);
  a78=(a78+a72);
  a29=(a29*a78);
  a86=(a86+a29);
  a86=(a1*a86);
  a16=(a16*a10);
  a29=(a2*a11);
  a29=(a37*a29);
  a16=(a16-a29);
  a29=(a11*a49);
  a29=(a34*a29);
  a16=(a16-a29);
  a16=(a16/a20);
  a29=(a86-a16);
  if (res[0]!=0) res[0][25]=a29;
  a19=(a49*a19);
  a19=(a1*a19);
  a19=(-a19);
  if (res[0]!=0) res[0][26]=a19;
  a29=(a1*a2);
  a29=(-a29);
  if (res[0]!=0) res[0][27]=a29;
  a78=(a49/a11);
  a78=(a1*a78);
  a78=(-a78);
  if (res[0]!=0) res[0][28]=a78;
  a72=(a32/a20);
  a68=(a37*a30);
  a57=(a26*a18);
  a68=(a68+a57);
  a57=(a36*a8);
  a68=(a68-a57);
  a57=(a26*a18);
  a68=(a68+a57);
  a57=(a5*a8);
  a68=(a68+a57);
  a68=(a1*a68);
  a57=(a72+a68);
  a57=(-a57);
  if (res[0]!=0) res[0][29]=a57;
  a57=(a24*a49);
  a53=(a8*a57);
  a31=(a30*a31);
  a53=(a53+a31);
  a31=(a32*a2);
  a52=(a8*a31);
  a53=(a53+a52);
  a89=(a30*a89);
  a53=(a53+a89);
  a32=(a32*a49);
  a32=(a17*a32);
  a53=(a53+a32);
  a53=(a1*a53);
  a32=(a5*a2);
  a89=(a26*a49);
  a32=(a32+a89);
  a32=(a32/a20);
  a89=(a53-a32);
  if (res[0]!=0) res[0][30]=a89;
  a94=(a11*a94);
  a89=(a8*a94);
  a15=(a30*a15);
  a89=(a89+a15);
  a48=(a2*a48);
  a15=(a8*a48);
  a89=(a89+a15);
  a54=(a30*a54);
  a89=(a89+a54);
  a83=(a83*a10);
  a54=(a8*a83);
  a89=(a89+a54);
  a66=(a30*a66);
  a89=(a89+a66);
  a66=(a2*a11);
  a66=(a34*a66);
  a54=(a26*a10);
  a66=(a66+a54);
  a17=(a17*a66);
  a89=(a89+a17);
  a89=(a1*a89);
  a17=(a34*a10);
  a66=(a2*a11);
  a66=(a26*a66);
  a17=(a17-a66);
  a66=(a11*a49);
  a66=(a5*a66);
  a17=(a17+a66);
  a17=(a17/a20);
  a66=(a89-a17);
  if (res[0]!=0) res[0][31]=a66;
  a12=(a2*a12);
  a12=(a1*a12);
  a12=(-a12);
  if (res[0]!=0) res[0][32]=a12;
  a66=(a1*a49);
  if (res[0]!=0) res[0][33]=a66;
  a54=(a2/a11);
  a54=(a1*a54);
  a54=(-a54);
  if (res[0]!=0) res[0][34]=a54;
  a15=(a36*a18);
  a34=(a34*a30);
  a5=(a5*a18);
  a34=(a34-a5);
  a5=(a26*a8);
  a34=(a34+a5);
  a15=(a15+a34);
  a8=(a26*a8);
  a15=(a15+a8);
  a15=(a1*a15);
  a8=(a24/a20);
  a34=(a15-a8);
  if (res[0]!=0) res[0][35]=a34;
  a57=(a18*a57);
  a76=(a30*a76);
  a57=(a57+a76);
  a31=(a18*a31);
  a57=(a57+a31);
  a39=(a30*a39);
  a57=(a57+a39);
  a39=(a24*a2);
  a39=(a7*a39);
  a57=(a57+a39);
  a57=(a1*a57);
  a39=(a41*a2);
  a31=(a36*a49);
  a39=(a39-a31);
  a39=(a39/a20);
  a31=(a57-a39);
  if (res[0]!=0) res[0][36]=a31;
  a94=(a18*a94);
  a28=(a30*a28);
  a94=(a94+a28);
  a48=(a18*a48);
  a94=(a94+a48);
  a51=(a30*a51);
  a94=(a94+a51);
  a18=(a18*a83);
  a94=(a94+a18);
  a30=(a30*a82);
  a94=(a94+a30);
  a30=(a11*a49);
  a24=(a24*a30);
  a41=(a41*a10);
  a24=(a24+a41);
  a7=(a7*a24);
  a94=(a94+a7);
  a94=(a1*a94);
  a37=(a37*a10);
  a2=(a2*a11);
  a36=(a36*a2);
  a37=(a37+a36);
  a49=(a11*a49);
  a26=(a26*a49);
  a37=(a37-a26);
  a37=(a37/a20);
  a26=(a94-a37);
  if (res[0]!=0) res[0][37]=a26;
  a26=7.2780203784570591e-002;
  a49=casadi_sq(a20);
  a36=(a1*a49);
  a36=(a26*a36);
  a36=(-a36);
  if (res[0]!=0) res[0][38]=a36;
  a2=(a26*a20);
  a2=(-a2);
  if (res[0]!=0) res[0][39]=a2;
  a7=(a75*a10);
  a44=(a44*a81);
  a7=(a7+a44);
  if (res[0]!=0) res[0][40]=a7;
  a7=(a93*a3);
  if (res[0]!=0) res[0][41]=a7;
  a7=(-a75);
  if (res[0]!=0) res[0][42]=a7;
  a7=1.;
  if (res[0]!=0) res[0][43]=a7;
  if (res[0]!=0) res[0][44]=a0;
  a44=(a1*a49);
  a44=(a26*a44);
  a44=(-a44);
  if (res[0]!=0) res[0][45]=a44;
  a24=(a26*a20);
  a24=(-a24);
  if (res[0]!=0) res[0][46]=a24;
  a41=(a77*a10);
  a55=(a55*a3);
  a41=(a41+a55);
  a41=(-a41);
  if (res[0]!=0) res[0][47]=a41;
  a93=(a93*a81);
  if (res[0]!=0) res[0][48]=a93;
  if (res[0]!=0) res[0][49]=a77;
  if (res[0]!=0) res[0][50]=a7;
  if (res[0]!=0) res[0][51]=a0;
  a1=(a1*a49);
  a1=(a26*a1);
  a1=(-a1);
  if (res[0]!=0) res[0][52]=a1;
  a26=(a26*a20);
  a26=(-a26);
  if (res[0]!=0) res[0][53]=a26;
  a91=(a11*a91);
  if (res[0]!=0) res[0][54]=a91;
  a77=(a77*a3);
  a75=(a75*a81);
  a77=(a77+a75);
  a77=(-a77);
  if (res[0]!=0) res[0][55]=a77;
  if (res[0]!=0) res[0][56]=a79;
  if (res[0]!=0) res[0][57]=a79;
  if (res[0]!=0) res[0][58]=a79;
  if (res[0]!=0) res[0][59]=a79;
  if (res[0]!=0) res[0][60]=a36;
  if (res[0]!=0) res[0][61]=a2;
  a77=(a73*a10);
  a35=(a35*a81);
  a77=(a77+a35);
  if (res[0]!=0) res[0][62]=a77;
  a77=(a90*a3);
  if (res[0]!=0) res[0][63]=a77;
  a77=(-a73);
  if (res[0]!=0) res[0][64]=a77;
  if (res[0]!=0) res[0][65]=a7;
  if (res[0]!=0) res[0][66]=a0;
  if (res[0]!=0) res[0][67]=a44;
  if (res[0]!=0) res[0][68]=a24;
  a77=(a71*a10);
  a45=(a45*a3);
  a77=(a77+a45);
  a77=(-a77);
  if (res[0]!=0) res[0][69]=a77;
  a90=(a90*a81);
  if (res[0]!=0) res[0][70]=a90;
  if (res[0]!=0) res[0][71]=a71;
  if (res[0]!=0) res[0][72]=a7;
  if (res[0]!=0) res[0][73]=a0;
  if (res[0]!=0) res[0][74]=a1;
  if (res[0]!=0) res[0][75]=a26;
  a88=(a11*a88);
  if (res[0]!=0) res[0][76]=a88;
  a71=(a71*a3);
  a73=(a73*a81);
  a71=(a71+a73);
  a71=(-a71);
  if (res[0]!=0) res[0][77]=a71;
  if (res[0]!=0) res[0][78]=a79;
  if (res[0]!=0) res[0][79]=a79;
  if (res[0]!=0) res[0][80]=a79;
  if (res[0]!=0) res[0][81]=a79;
  if (res[0]!=0) res[0][82]=a36;
  if (res[0]!=0) res[0][83]=a2;
  a71=(a69*a10);
  a56=(a56*a81);
  a71=(a71+a56);
  if (res[0]!=0) res[0][84]=a71;
  a71=(a87*a3);
  if (res[0]!=0) res[0][85]=a71;
  a71=(-a69);
  if (res[0]!=0) res[0][86]=a71;
  if (res[0]!=0) res[0][87]=a7;
  if (res[0]!=0) res[0][88]=a0;
  if (res[0]!=0) res[0][89]=a44;
  if (res[0]!=0) res[0][90]=a24;
  a71=(a67*a10);
  a46=(a46*a3);
  a71=(a71+a46);
  a71=(-a71);
  if (res[0]!=0) res[0][91]=a71;
  a87=(a87*a81);
  if (res[0]!=0) res[0][92]=a87;
  if (res[0]!=0) res[0][93]=a67;
  if (res[0]!=0) res[0][94]=a7;
  if (res[0]!=0) res[0][95]=a0;
  if (res[0]!=0) res[0][96]=a1;
  if (res[0]!=0) res[0][97]=a26;
  a85=(a11*a85);
  if (res[0]!=0) res[0][98]=a85;
  a67=(a67*a3);
  a69=(a69*a81);
  a67=(a67+a69);
  a67=(-a67);
  if (res[0]!=0) res[0][99]=a67;
  if (res[0]!=0) res[0][100]=a79;
  if (res[0]!=0) res[0][101]=a79;
  if (res[0]!=0) res[0][102]=a79;
  if (res[0]!=0) res[0][103]=a79;
  if (res[0]!=0) res[0][104]=a36;
  if (res[0]!=0) res[0][105]=a2;
  a2=(a61*a10);
  a58=(a58*a81);
  a2=(a2+a58);
  if (res[0]!=0) res[0][106]=a2;
  a2=(a84*a3);
  if (res[0]!=0) res[0][107]=a2;
  a2=(-a61);
  if (res[0]!=0) res[0][108]=a2;
  if (res[0]!=0) res[0][109]=a7;
  if (res[0]!=0) res[0][110]=a0;
  if (res[0]!=0) res[0][111]=a44;
  if (res[0]!=0) res[0][112]=a24;
  a10=(a65*a10);
  a43=(a43*a3);
  a10=(a10+a43);
  a10=(-a10);
  if (res[0]!=0) res[0][113]=a10;
  a84=(a84*a81);
  if (res[0]!=0) res[0][114]=a84;
  if (res[0]!=0) res[0][115]=a65;
  if (res[0]!=0) res[0][116]=a7;
  if (res[0]!=0) res[0][117]=a0;
  if (res[0]!=0) res[0][118]=a1;
  if (res[0]!=0) res[0][119]=a26;
  a11=(a11*a80);
  if (res[0]!=0) res[0][120]=a11;
  a65=(a65*a3);
  a61=(a61*a81);
  a65=(a65+a61);
  a65=(-a65);
  if (res[0]!=0) res[0][121]=a65;
  if (res[0]!=0) res[0][122]=a79;
  if (res[0]!=0) res[0][123]=a79;
  if (res[0]!=0) res[0][124]=a79;
  if (res[0]!=0) res[0][125]=a79;
  if (res[0]!=0) res[0][126]=a7;
  if (res[0]!=0) res[0][127]=a7;
  if (res[0]!=0) res[0][128]=a7;
  a13=(a13+a21);
  if (res[0]!=0) res[0][129]=a13;
  if (res[0]!=0) res[0][130]=a22;
  if (res[0]!=0) res[0][131]=a23;
  if (res[0]!=0) res[0][132]=a25;
  if (res[0]!=0) res[0][133]=a14;
  if (res[0]!=0) res[0][134]=a60;
  if (res[0]!=0) res[0][135]=a62;
  if (res[0]!=0) res[0][136]=a59;
  if (res[0]!=0) res[0][137]=a64;
  if (res[0]!=0) res[0][138]=a4;
  if (res[0]!=0) res[0][139]=a9;
  if (res[0]!=0) res[0][140]=a47;
  if (res[0]!=0) res[0][141]=a38;
  if (res[0]!=0) res[0][142]=a7;
  if (res[0]!=0) res[0][143]=a7;
  if (res[0]!=0) res[0][144]=a7;
  if (res[0]!=0) res[0][145]=a79;
  a92=(a92+a63);
  if (res[0]!=0) res[0][146]=a92;
  a70=(a70+a74);
  if (res[0]!=0) res[0][147]=a70;
  a16=(a16+a86);
  if (res[0]!=0) res[0][148]=a16;
  if (res[0]!=0) res[0][149]=a19;
  if (res[0]!=0) res[0][150]=a29;
  if (res[0]!=0) res[0][151]=a78;
  a72=(a72-a68);
  if (res[0]!=0) res[0][152]=a72;
  a32=(a32+a53);
  if (res[0]!=0) res[0][153]=a32;
  a17=(a17+a89);
  if (res[0]!=0) res[0][154]=a17;
  if (res[0]!=0) res[0][155]=a12;
  if (res[0]!=0) res[0][156]=a66;
  if (res[0]!=0) res[0][157]=a54;
  a8=(a8+a15);
  if (res[0]!=0) res[0][158]=a8;
  a39=(a39+a57);
  if (res[0]!=0) res[0][159]=a39;
  a37=(a37+a94);
  if (res[0]!=0) res[0][160]=a37;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_jac_g_leg_n_in(void) { return 2;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_jac_g_leg_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real eval_jac_g_leg_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_jac_g_leg_name_in(casadi_int i){
  switch (i) {
    case 0: return "w";
    case 1: return "p";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_jac_g_leg_name_out(casadi_int i){
  switch (i) {
    case 0: return "jac_g";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_jac_g_leg_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_jac_g_leg_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


