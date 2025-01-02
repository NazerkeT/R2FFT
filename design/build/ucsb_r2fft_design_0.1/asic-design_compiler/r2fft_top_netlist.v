/////////////////////////////////////////////////////////////
// Created by: Synopsys DC Ultra(TM) in wire load mode
// Version   : R-2020.09-SP4
// Date      : Thu Jan  2 14:46:36 2025
/////////////////////////////////////////////////////////////


module bitReverseCounter_BIT_WIDTH10 ( rst, clk, clr, inc, iter, count, 
        countFull );
  output [9:0] iter;
  output [9:0] count;
  input rst, clk, clr, inc;
  output countFull;
  wire   n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n1, n2, n3, n4, n5,
         n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50;
  tri   rst;
  tri   clk;
  assign count[9] = iter[0];
  assign count[8] = iter[1];
  assign count[7] = iter[2];
  assign count[6] = iter[3];
  assign count[5] = iter[4];
  assign count[4] = iter[5];
  assign count[3] = iter[6];
  assign count[2] = iter[7];
  assign count[1] = iter[8];
  assign count[0] = iter[9];

  DFF_X1 \ptr_f_reg[9]  ( .D(n17), .CK(clk), .Q(iter[0]), .QN(n47) );
  DFF_X1 \ptr_f_reg[8]  ( .D(n18), .CK(clk), .Q(iter[1]) );
  DFF_X1 \ptr_f_reg[7]  ( .D(n19), .CK(clk), .Q(iter[2]), .QN(n46) );
  DFF_X1 \ptr_f_reg[6]  ( .D(n20), .CK(clk), .Q(iter[3]), .QN(n50) );
  DFF_X1 \ptr_f_reg[5]  ( .D(n21), .CK(clk), .Q(iter[4]), .QN(n45) );
  DFF_X1 \ptr_f_reg[4]  ( .D(n22), .CK(clk), .Q(iter[5]), .QN(n49) );
  DFF_X1 \ptr_f_reg[3]  ( .D(n23), .CK(clk), .Q(iter[6]), .QN(n44) );
  DFF_X1 \ptr_f_reg[2]  ( .D(n24), .CK(clk), .Q(iter[7]) );
  DFF_X1 \ptr_f_reg[1]  ( .D(n25), .CK(clk), .Q(iter[8]), .QN(n48) );
  DFF_X1 \ptr_f_reg[0]  ( .D(n26), .CK(clk), .Q(iter[9]), .QN(n43) );
  NOR2_X1 U3 ( .A1(clr), .A2(rst), .ZN(n33) );
  NAND2_X1 U4 ( .A1(n33), .A2(inc), .ZN(n13) );
  INV_X1 U5 ( .A(n13), .ZN(n39) );
  NAND3_X1 U6 ( .A1(iter[9]), .A2(iter[8]), .A3(iter[7]), .ZN(n27) );
  NOR2_X1 U7 ( .A1(n27), .A2(n44), .ZN(n28) );
  NAND2_X1 U8 ( .A1(iter[5]), .A2(n28), .ZN(n34) );
  NOR2_X1 U9 ( .A1(n34), .A2(n45), .ZN(n35) );
  NAND2_X1 U10 ( .A1(iter[3]), .A2(n35), .ZN(n40) );
  NOR3_X1 U11 ( .A1(clr), .A2(rst), .A3(inc), .ZN(n32) );
  AOI21_X1 U12 ( .B1(n39), .B2(n40), .A(n32), .ZN(n42) );
  OAI21_X1 U13 ( .B1(iter[2]), .B2(n13), .A(n42), .ZN(n3) );
  NOR2_X1 U14 ( .A1(iter[1]), .A2(n13), .ZN(n4) );
  NOR2_X1 U15 ( .A1(n46), .A2(n40), .ZN(n2) );
  AOI22_X1 U16 ( .A1(iter[1]), .A2(n3), .B1(n4), .B2(n2), .ZN(n1) );
  INV_X1 U17 ( .A(n1), .ZN(n18) );
  NAND2_X1 U18 ( .A1(iter[1]), .A2(n2), .ZN(n12) );
  NOR2_X1 U19 ( .A1(n13), .A2(n12), .ZN(n6) );
  OR2_X1 U20 ( .A1(n4), .A2(n3), .ZN(n5) );
  MUX2_X1 U21 ( .A(n6), .B(n5), .S(iter[0]), .Z(n17) );
  NAND2_X1 U22 ( .A1(iter[9]), .A2(iter[8]), .ZN(n7) );
  NOR2_X1 U23 ( .A1(n13), .A2(n7), .ZN(n11) );
  AOI21_X1 U24 ( .B1(n43), .B2(n39), .A(n32), .ZN(n16) );
  INV_X1 U25 ( .A(n16), .ZN(n8) );
  AOI21_X1 U26 ( .B1(n39), .B2(n48), .A(n8), .ZN(n9) );
  INV_X1 U27 ( .A(n9), .ZN(n10) );
  MUX2_X1 U28 ( .A(n11), .B(n10), .S(iter[7]), .Z(n24) );
  NOR2_X1 U29 ( .A1(n47), .A2(n12), .ZN(countFull) );
  INV_X1 U30 ( .A(n32), .ZN(n14) );
  AOI22_X1 U31 ( .A1(iter[9]), .A2(n14), .B1(n13), .B2(n43), .ZN(n26) );
  NAND2_X1 U32 ( .A1(n39), .A2(n48), .ZN(n15) );
  OAI22_X1 U33 ( .A1(n16), .A2(n48), .B1(n43), .B2(n15), .ZN(n25) );
  AOI21_X1 U34 ( .B1(n33), .B2(n27), .A(n32), .ZN(n31) );
  NAND2_X1 U35 ( .A1(n39), .A2(n44), .ZN(n30) );
  OAI22_X1 U36 ( .A1(n31), .A2(n44), .B1(n27), .B2(n30), .ZN(n23) );
  NAND3_X1 U37 ( .A1(n39), .A2(n28), .A3(n49), .ZN(n29) );
  OAI221_X1 U38 ( .B1(n49), .B2(n31), .C1(n49), .C2(n30), .A(n29), .ZN(n22) );
  AOI21_X1 U39 ( .B1(n33), .B2(n34), .A(n32), .ZN(n38) );
  NAND2_X1 U40 ( .A1(n39), .A2(n45), .ZN(n37) );
  OAI22_X1 U41 ( .A1(n38), .A2(n45), .B1(n34), .B2(n37), .ZN(n21) );
  NAND3_X1 U42 ( .A1(n39), .A2(n35), .A3(n50), .ZN(n36) );
  OAI221_X1 U43 ( .B1(n50), .B2(n38), .C1(n50), .C2(n37), .A(n36), .ZN(n20) );
  NAND2_X1 U44 ( .A1(n39), .A2(n46), .ZN(n41) );
  OAI22_X1 U45 ( .A1(n42), .A2(n46), .B1(n41), .B2(n40), .ZN(n19) );
endmodule


module bfp_bitWidthDetector_FFT_BFPDW5_FFT_DW16_1 ( operand0, operand1, 
        operand2, operand3, bw );
  input [15:0] operand0;
  input [15:0] operand1;
  input [15:0] operand2;
  input [15:0] operand3;
  output [4:0] bw;
  wire   n157, N51, N84, n36, n37, n38, n39, n40, n41, n42, n43, n44, n45, n46,
         n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58, n59, n60,
         n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72, n73, n74,
         n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86, n87, n88,
         n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100, n101,
         n102, n103, n104, n105, n106, n107, n108, n109, n110, n111, n112,
         n113, n114, n115, n116, n117, n118, n119, n120, n121, n122, n123,
         n124, n125, n126, n127, n128, n129, n130, n131, n132, n133, n134,
         n135, n136, n137, n138, n139, n140, n141, n142, n143, n144, n145,
         n146, n147, n148, n149, n150, n151, n152, n153, n154, n155, n156;
  assign N51 = operand0[0];
  assign N84 = operand1[0];

  CLKBUF_X1 U3 ( .A(n157), .Z(bw[4]) );
  AOI21_X1 U4 ( .B1(n127), .B2(n126), .A(bw[4]), .ZN(bw[1]) );
  NOR4_X1 U5 ( .A1(operand1[3]), .A2(operand1[2]), .A3(operand1[1]), .A4(N84), 
        .ZN(n92) );
  OR2_X1 U6 ( .A1(operand1[11]), .A2(n50), .ZN(n47) );
  NOR2_X1 U7 ( .A1(operand1[5]), .A2(n87), .ZN(n73) );
  INV_X1 U8 ( .A(operand1[15]), .ZN(n118) );
  INV_X1 U9 ( .A(operand1[4]), .ZN(n94) );
  NAND2_X1 U10 ( .A1(n92), .A2(n94), .ZN(n87) );
  INV_X1 U11 ( .A(operand1[6]), .ZN(n72) );
  NAND2_X1 U12 ( .A1(n73), .A2(n72), .ZN(n74) );
  OR3_X1 U13 ( .A1(operand1[7]), .A2(operand1[8]), .A3(n74), .ZN(n60) );
  NOR2_X1 U14 ( .A1(operand1[9]), .A2(n60), .ZN(n55) );
  INV_X1 U15 ( .A(operand1[10]), .ZN(n36) );
  NAND2_X1 U16 ( .A1(n55), .A2(n36), .ZN(n42) );
  NOR3_X1 U17 ( .A1(operand1[12]), .A2(operand1[11]), .A3(n42), .ZN(n119) );
  INV_X1 U18 ( .A(operand1[13]), .ZN(n121) );
  NAND2_X1 U19 ( .A1(n119), .A2(n121), .ZN(n112) );
  NOR4_X1 U20 ( .A1(operand0[3]), .A2(operand0[2]), .A3(operand0[1]), .A4(N51), 
        .ZN(n91) );
  INV_X1 U21 ( .A(operand0[4]), .ZN(n97) );
  NAND2_X1 U22 ( .A1(n91), .A2(n97), .ZN(n86) );
  NOR2_X1 U23 ( .A1(operand0[5]), .A2(n86), .ZN(n67) );
  INV_X1 U24 ( .A(operand0[6]), .ZN(n66) );
  NAND2_X1 U25 ( .A1(n67), .A2(n66), .ZN(n68) );
  NOR3_X1 U26 ( .A1(operand0[7]), .A2(operand0[8]), .A3(n68), .ZN(n59) );
  INV_X1 U27 ( .A(operand0[9]), .ZN(n64) );
  NAND2_X1 U28 ( .A1(n59), .A2(n64), .ZN(n54) );
  NOR2_X1 U29 ( .A1(operand0[10]), .A2(n54), .ZN(n51) );
  INV_X1 U30 ( .A(operand0[11]), .ZN(n37) );
  NAND2_X1 U31 ( .A1(n51), .A2(n37), .ZN(n43) );
  NOR2_X1 U32 ( .A1(operand0[12]), .A2(n43), .ZN(n117) );
  INV_X1 U33 ( .A(operand0[13]), .ZN(n124) );
  NAND2_X1 U34 ( .A1(n117), .A2(n124), .ZN(n111) );
  INV_X1 U35 ( .A(operand0[15]), .ZN(n116) );
  OAI33_X1 U36 ( .A1(n118), .A2(operand1[14]), .A3(n112), .B1(n111), .B2(
        operand0[14]), .B3(n116), .ZN(n157) );
  NAND2_X1 U37 ( .A1(operand0[6]), .A2(operand0[15]), .ZN(n38) );
  OAI22_X1 U38 ( .A1(n67), .A2(n38), .B1(operand0[6]), .B2(operand0[15]), .ZN(
        n41) );
  NAND2_X1 U39 ( .A1(operand1[6]), .A2(operand1[15]), .ZN(n39) );
  OAI22_X1 U40 ( .A1(n73), .A2(n39), .B1(operand1[6]), .B2(operand1[15]), .ZN(
        n40) );
  NAND2_X1 U41 ( .A1(n41), .A2(n40), .ZN(n150) );
  NAND2_X1 U42 ( .A1(operand1[15]), .A2(n42), .ZN(n50) );
  OAI21_X1 U43 ( .B1(operand1[11]), .B2(n42), .A(operand1[15]), .ZN(n46) );
  NAND2_X1 U44 ( .A1(operand0[15]), .A2(n43), .ZN(n45) );
  AOI22_X1 U45 ( .A1(operand1[12]), .A2(n46), .B1(operand0[12]), .B2(n45), 
        .ZN(n44) );
  OAI221_X1 U46 ( .B1(operand1[12]), .B2(n46), .C1(n45), .C2(operand0[12]), 
        .A(n44), .ZN(n143) );
  INV_X1 U47 ( .A(n143), .ZN(n48) );
  NAND2_X1 U48 ( .A1(n48), .A2(n47), .ZN(n49) );
  AOI21_X1 U49 ( .B1(operand1[11]), .B2(n50), .A(n49), .ZN(n53) );
  OAI21_X1 U50 ( .B1(n116), .B2(n51), .A(operand0[11]), .ZN(n52) );
  OAI211_X1 U51 ( .C1(n116), .C2(operand0[11]), .A(n53), .B(n52), .ZN(n147) );
  NAND2_X1 U52 ( .A1(operand0[15]), .A2(n54), .ZN(n58) );
  OR2_X1 U53 ( .A1(n55), .A2(n118), .ZN(n57) );
  OAI22_X1 U54 ( .A1(operand0[10]), .A2(n58), .B1(n57), .B2(operand1[10]), 
        .ZN(n56) );
  AOI221_X1 U55 ( .B1(n58), .B2(operand0[10]), .C1(n57), .C2(operand1[10]), 
        .A(n56), .ZN(n128) );
  NOR2_X1 U56 ( .A1(n59), .A2(n116), .ZN(n63) );
  NAND2_X1 U57 ( .A1(operand1[15]), .A2(n60), .ZN(n62) );
  OAI22_X1 U58 ( .A1(n64), .A2(n63), .B1(n62), .B2(operand1[9]), .ZN(n61) );
  AOI221_X1 U59 ( .B1(n64), .B2(n63), .C1(operand1[9]), .C2(n62), .A(n61), 
        .ZN(n65) );
  NAND2_X1 U60 ( .A1(n128), .A2(n65), .ZN(n154) );
  NOR2_X1 U61 ( .A1(n147), .A2(n154), .ZN(n131) );
  AOI21_X1 U62 ( .B1(n67), .B2(n66), .A(n116), .ZN(n79) );
  INV_X1 U63 ( .A(n79), .ZN(n71) );
  INV_X1 U64 ( .A(operand0[7]), .ZN(n70) );
  NOR2_X1 U65 ( .A1(n70), .A2(n116), .ZN(n69) );
  AOI22_X1 U66 ( .A1(n71), .A2(n70), .B1(n69), .B2(n68), .ZN(n84) );
  AOI21_X1 U67 ( .B1(n73), .B2(n72), .A(n118), .ZN(n78) );
  INV_X1 U68 ( .A(n78), .ZN(n77) );
  INV_X1 U69 ( .A(operand1[7]), .ZN(n76) );
  NOR2_X1 U70 ( .A1(n76), .A2(n118), .ZN(n75) );
  AOI22_X1 U71 ( .A1(n77), .A2(n76), .B1(n75), .B2(n74), .ZN(n83) );
  AOI21_X1 U72 ( .B1(operand1[7]), .B2(operand1[15]), .A(n78), .ZN(n82) );
  AOI21_X1 U73 ( .B1(operand0[7]), .B2(operand0[15]), .A(n79), .ZN(n81) );
  AOI22_X1 U74 ( .A1(operand1[8]), .A2(n82), .B1(operand0[8]), .B2(n81), .ZN(
        n80) );
  OAI221_X1 U75 ( .B1(operand1[8]), .B2(n82), .C1(operand0[8]), .C2(n81), .A(
        n80), .ZN(n130) );
  NOR3_X1 U76 ( .A1(n84), .A2(n83), .A3(n130), .ZN(n155) );
  NAND2_X1 U77 ( .A1(n131), .A2(n155), .ZN(n138) );
  INV_X1 U78 ( .A(n138), .ZN(n148) );
  OR3_X1 U79 ( .A1(operand1[2]), .A2(operand1[1]), .A3(N84), .ZN(n85) );
  NAND2_X1 U80 ( .A1(operand1[15]), .A2(n85), .ZN(n103) );
  NAND2_X1 U81 ( .A1(operand0[15]), .A2(n86), .ZN(n90) );
  NAND2_X1 U82 ( .A1(operand1[15]), .A2(n87), .ZN(n89) );
  OAI22_X1 U83 ( .A1(operand0[5]), .A2(n90), .B1(n89), .B2(operand1[5]), .ZN(
        n88) );
  AOI221_X1 U84 ( .B1(n90), .B2(operand0[5]), .C1(n89), .C2(operand1[5]), .A(
        n88), .ZN(n137) );
  NOR2_X1 U85 ( .A1(n91), .A2(n116), .ZN(n96) );
  NOR2_X1 U86 ( .A1(n92), .A2(n118), .ZN(n95) );
  OAI22_X1 U87 ( .A1(n97), .A2(n96), .B1(n94), .B2(n95), .ZN(n93) );
  AOI221_X1 U88 ( .B1(n97), .B2(n96), .C1(n95), .C2(n94), .A(n93), .ZN(n98) );
  NAND2_X1 U89 ( .A1(n137), .A2(n98), .ZN(n136) );
  NOR3_X1 U90 ( .A1(operand0[2]), .A2(operand0[1]), .A3(N51), .ZN(n99) );
  OAI21_X1 U91 ( .B1(n99), .B2(n116), .A(operand0[3]), .ZN(n100) );
  OAI21_X1 U92 ( .B1(operand0[3]), .B2(n116), .A(n100), .ZN(n101) );
  AOI211_X1 U93 ( .C1(n103), .C2(operand1[3]), .A(n136), .B(n101), .ZN(n102)
         );
  OAI21_X1 U94 ( .B1(n103), .B2(operand1[3]), .A(n102), .ZN(n149) );
  OAI21_X1 U95 ( .B1(operand0[1]), .B2(N51), .A(operand0[15]), .ZN(n106) );
  OAI21_X1 U96 ( .B1(operand1[1]), .B2(N84), .A(operand1[15]), .ZN(n105) );
  OAI22_X1 U97 ( .A1(operand0[2]), .A2(n106), .B1(n105), .B2(operand1[2]), 
        .ZN(n104) );
  AOI221_X1 U98 ( .B1(n106), .B2(operand0[2]), .C1(n105), .C2(operand1[2]), 
        .A(n104), .ZN(n134) );
  NAND2_X1 U99 ( .A1(N51), .A2(operand0[15]), .ZN(n110) );
  INV_X1 U100 ( .A(operand1[1]), .ZN(n109) );
  AND2_X1 U101 ( .A1(N84), .A2(operand1[15]), .ZN(n108) );
  OAI22_X1 U102 ( .A1(operand0[1]), .A2(n110), .B1(n109), .B2(n108), .ZN(n107)
         );
  AOI221_X1 U103 ( .B1(n110), .B2(operand0[1]), .C1(n109), .C2(n108), .A(n107), 
        .ZN(n132) );
  OAI221_X1 U104 ( .B1(n149), .B2(n134), .C1(n149), .C2(n132), .A(n137), .ZN(
        n125) );
  NAND2_X1 U105 ( .A1(operand0[15]), .A2(n111), .ZN(n115) );
  NAND2_X1 U106 ( .A1(operand1[15]), .A2(n112), .ZN(n114) );
  OAI22_X1 U107 ( .A1(operand0[14]), .A2(n115), .B1(n114), .B2(operand1[14]), 
        .ZN(n113) );
  AOI221_X1 U108 ( .B1(n115), .B2(operand0[14]), .C1(n114), .C2(operand1[14]), 
        .A(n113), .ZN(n145) );
  NOR2_X1 U109 ( .A1(n117), .A2(n116), .ZN(n123) );
  NOR2_X1 U110 ( .A1(n119), .A2(n118), .ZN(n122) );
  OAI22_X1 U111 ( .A1(n124), .A2(n123), .B1(n121), .B2(n122), .ZN(n120) );
  AOI221_X1 U112 ( .B1(n124), .B2(n123), .C1(n122), .C2(n121), .A(n120), .ZN(
        n141) );
  NAND2_X1 U113 ( .A1(n145), .A2(n141), .ZN(n146) );
  AOI221_X1 U114 ( .B1(n150), .B2(n148), .C1(n125), .C2(n148), .A(n146), .ZN(
        n127) );
  OR2_X1 U115 ( .A1(n147), .A2(n131), .ZN(n126) );
  INV_X1 U116 ( .A(n128), .ZN(n129) );
  AOI21_X1 U117 ( .B1(n131), .B2(n130), .A(n129), .ZN(n140) );
  OAI21_X1 U118 ( .B1(N51), .B2(N84), .A(n132), .ZN(n133) );
  AOI21_X1 U119 ( .B1(n134), .B2(n133), .A(n149), .ZN(n135) );
  AOI211_X1 U120 ( .C1(n137), .C2(n136), .A(n135), .B(n150), .ZN(n139) );
  OAI22_X1 U121 ( .A1(n140), .A2(n147), .B1(n139), .B2(n138), .ZN(n142) );
  OAI21_X1 U122 ( .B1(n143), .B2(n142), .A(n141), .ZN(n144) );
  AOI21_X1 U123 ( .B1(n145), .B2(n144), .A(bw[4]), .ZN(bw[0]) );
  NOR2_X1 U124 ( .A1(n147), .A2(n146), .ZN(n152) );
  OAI21_X1 U125 ( .B1(n150), .B2(n149), .A(n148), .ZN(n151) );
  AOI21_X1 U126 ( .B1(n152), .B2(n151), .A(bw[4]), .ZN(bw[2]) );
  INV_X1 U127 ( .A(n152), .ZN(n153) );
  NOR2_X1 U128 ( .A1(n154), .A2(n153), .ZN(n156) );
  AOI21_X1 U129 ( .B1(n156), .B2(n155), .A(bw[4]), .ZN(bw[3]) );
endmodule


module bfp_maxBitWidth_FFT_BFPDW5_1 ( rst, clk, clr, bw_act, bw, max_bw );
  input [4:0] bw;
  output [4:0] max_bw;
  input rst, clk, clr, bw_act;
  wire   n23, n24, n25, n26, n27, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11,
         n12, n13, n14, n15, n16, n17, n18, n19, n20, n21, n22;
  tri   rst;
  tri   clk;

  DFF_X1 \max_bw_f_reg[4]  ( .D(n23), .CK(clk), .Q(max_bw[4]), .QN(n19) );
  DFF_X1 \max_bw_f_reg[3]  ( .D(n24), .CK(clk), .Q(max_bw[3]), .QN(n21) );
  DFF_X1 \max_bw_f_reg[2]  ( .D(n25), .CK(clk), .Q(max_bw[2]), .QN(n20) );
  DFF_X1 \max_bw_f_reg[1]  ( .D(n26), .CK(clk), .Q(max_bw[1]), .QN(n22) );
  DFF_X1 \max_bw_f_reg[0]  ( .D(n27), .CK(clk), .Q(max_bw[0]) );
  OR2_X1 U3 ( .A1(n21), .A2(bw[3]), .ZN(n3) );
  AOI22_X1 U4 ( .A1(bw[4]), .A2(n19), .B1(bw[3]), .B2(n21), .ZN(n7) );
  INV_X1 U5 ( .A(bw[1]), .ZN(n14) );
  OAI222_X1 U6 ( .A1(n14), .A2(max_bw[1]), .B1(n14), .B2(max_bw[0]), .C1(
        max_bw[1]), .C2(max_bw[0]), .ZN(n1) );
  OAI222_X1 U7 ( .A1(n20), .A2(n1), .B1(n20), .B2(bw[2]), .C1(n1), .C2(bw[2]), 
        .ZN(n2) );
  INV_X1 U8 ( .A(n2), .ZN(n4) );
  NAND2_X1 U9 ( .A1(n4), .A2(n3), .ZN(n6) );
  OAI21_X1 U10 ( .B1(bw[4]), .B2(n19), .A(bw_act), .ZN(n5) );
  AOI21_X1 U11 ( .B1(n7), .B2(n6), .A(n5), .ZN(n8) );
  NOR3_X1 U12 ( .A1(n8), .A2(rst), .A3(clr), .ZN(n12) );
  INV_X1 U13 ( .A(n8), .ZN(n9) );
  NOR3_X1 U14 ( .A1(rst), .A2(clr), .A3(n9), .ZN(n13) );
  AOI22_X1 U15 ( .A1(max_bw[0]), .A2(n12), .B1(n13), .B2(bw[0]), .ZN(n10) );
  INV_X1 U16 ( .A(n10), .ZN(n27) );
  AOI22_X1 U17 ( .A1(max_bw[3]), .A2(n12), .B1(bw[3]), .B2(n13), .ZN(n11) );
  INV_X1 U18 ( .A(n11), .ZN(n24) );
  INV_X1 U19 ( .A(n12), .ZN(n18) );
  INV_X1 U20 ( .A(n13), .ZN(n16) );
  OAI22_X1 U21 ( .A1(n22), .A2(n18), .B1(n14), .B2(n16), .ZN(n26) );
  INV_X1 U22 ( .A(bw[2]), .ZN(n15) );
  OAI22_X1 U23 ( .A1(n20), .A2(n18), .B1(n15), .B2(n16), .ZN(n25) );
  INV_X1 U24 ( .A(bw[4]), .ZN(n17) );
  OAI22_X1 U25 ( .A1(n19), .A2(n18), .B1(n17), .B2(n16), .ZN(n23) );
endmodule


module bfp_bitWidthAcc_FFT_BFPDW5_FFT_DW16 ( clk, rst, init, bw_init, update, 
        bw_new, bfp_bw, bfp_exponent );
  input [4:0] bw_init;
  input [4:0] bw_new;
  output [4:0] bfp_bw;
  output [7:0] bfp_exponent;
  input clk, rst, init, update;
  wire   n37, n38, n39, n40, n41, n42, n43, n44, n45, n46, n47, n48, n49, n1,
         n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n50, n51, n52, n53, n54, n55, n56, n57,
         n58, n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71,
         n72, n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85,
         n86, n87, n88, n89, n90, n91, n92, n93;
  tri   clk;
  tri   rst;

  DFF_X1 \bfp_bw_f_reg[4]  ( .D(n37), .CK(clk), .Q(bfp_bw[4]) );
  DFF_X1 \bfp_bw_f_reg[3]  ( .D(n38), .CK(clk), .Q(bfp_bw[3]) );
  DFF_X1 \bfp_bw_f_reg[2]  ( .D(n39), .CK(clk), .Q(bfp_bw[2]) );
  DFF_X1 \bfp_bw_f_reg[1]  ( .D(n40), .CK(clk), .Q(bfp_bw[1]) );
  DFF_X1 \bfp_bw_f_reg[0]  ( .D(n41), .CK(clk), .Q(bfp_bw[0]) );
  DFF_X1 \bfp_exponent_signed_reg[7]  ( .D(n42), .CK(clk), .Q(bfp_exponent[7]), 
        .QN(n93) );
  DFF_X1 \bfp_exponent_signed_reg[6]  ( .D(n43), .CK(clk), .Q(bfp_exponent[6]), 
        .QN(n88) );
  DFF_X1 \bfp_exponent_signed_reg[5]  ( .D(n44), .CK(clk), .Q(bfp_exponent[5]), 
        .QN(n91) );
  DFF_X1 \bfp_exponent_signed_reg[4]  ( .D(n45), .CK(clk), .Q(bfp_exponent[4]), 
        .QN(n90) );
  DFF_X1 \bfp_exponent_signed_reg[3]  ( .D(n46), .CK(clk), .Q(bfp_exponent[3])
         );
  DFF_X1 \bfp_exponent_signed_reg[2]  ( .D(n47), .CK(clk), .Q(bfp_exponent[2]), 
        .QN(n89) );
  DFF_X1 \bfp_exponent_signed_reg[1]  ( .D(n48), .CK(clk), .Q(bfp_exponent[1]), 
        .QN(n92) );
  DFF_X1 \bfp_exponent_signed_reg[0]  ( .D(n49), .CK(clk), .Q(bfp_exponent[0])
         );
  NOR3_X2 U3 ( .A1(rst), .A2(init), .A3(n56), .ZN(n83) );
  NOR3_X2 U4 ( .A1(rst), .A2(init), .A3(update), .ZN(n56) );
  INV_X1 U5 ( .A(init), .ZN(n6) );
  NOR2_X1 U6 ( .A1(rst), .A2(n6), .ZN(n65) );
  AOI222_X1 U7 ( .A1(n56), .A2(bfp_bw[0]), .B1(n83), .B2(bw_new[0]), .C1(
        bw_init[0]), .C2(n65), .ZN(n1) );
  INV_X1 U8 ( .A(n1), .ZN(n41) );
  AOI222_X1 U9 ( .A1(n56), .A2(bfp_bw[1]), .B1(n83), .B2(bw_new[1]), .C1(
        bw_init[1]), .C2(n65), .ZN(n2) );
  INV_X1 U10 ( .A(n2), .ZN(n40) );
  AOI222_X1 U11 ( .A1(n56), .A2(bfp_bw[3]), .B1(n83), .B2(bw_new[3]), .C1(
        bw_init[3]), .C2(n65), .ZN(n3) );
  INV_X1 U12 ( .A(n3), .ZN(n38) );
  AOI222_X1 U13 ( .A1(n56), .A2(bfp_bw[2]), .B1(n83), .B2(bw_new[2]), .C1(
        bw_init[2]), .C2(n65), .ZN(n4) );
  INV_X1 U14 ( .A(n4), .ZN(n39) );
  AOI222_X1 U15 ( .A1(n56), .A2(bfp_bw[4]), .B1(n83), .B2(bw_new[4]), .C1(
        bw_init[4]), .C2(n65), .ZN(n5) );
  INV_X1 U16 ( .A(n5), .ZN(n37) );
  INV_X1 U17 ( .A(n83), .ZN(n33) );
  AOI22_X1 U18 ( .A1(init), .A2(bw_init[4]), .B1(bw_new[4]), .B2(n6), .ZN(n60)
         );
  AOI22_X1 U19 ( .A1(init), .A2(bw_init[2]), .B1(bw_new[2]), .B2(n6), .ZN(n15)
         );
  AOI22_X1 U20 ( .A1(init), .A2(bw_init[1]), .B1(bw_new[1]), .B2(n6), .ZN(n14)
         );
  AND2_X1 U21 ( .A1(n15), .A2(n14), .ZN(n16) );
  AOI22_X1 U22 ( .A1(init), .A2(bw_init[3]), .B1(bw_new[3]), .B2(n6), .ZN(n34)
         );
  AOI22_X1 U23 ( .A1(init), .A2(bw_init[0]), .B1(bw_new[0]), .B2(n6), .ZN(n7)
         );
  NAND3_X1 U24 ( .A1(n16), .A2(n34), .A3(n7), .ZN(n64) );
  OAI21_X1 U25 ( .B1(n60), .B2(n64), .A(n7), .ZN(n11) );
  INV_X1 U26 ( .A(n56), .ZN(n87) );
  OAI21_X1 U27 ( .B1(n33), .B2(n11), .A(n87), .ZN(n9) );
  INV_X1 U28 ( .A(n65), .ZN(n58) );
  OAI21_X1 U29 ( .B1(bfp_exponent[0]), .B2(n33), .A(n58), .ZN(n8) );
  AOI22_X1 U30 ( .A1(bfp_exponent[0]), .A2(n9), .B1(n11), .B2(n8), .ZN(n10) );
  INV_X1 U31 ( .A(n10), .ZN(n49) );
  AND2_X1 U32 ( .A1(n64), .A2(n14), .ZN(n18) );
  AND2_X1 U33 ( .A1(bfp_exponent[0]), .A2(n11), .ZN(n17) );
  AOI22_X1 U34 ( .A1(n83), .A2(n12), .B1(n65), .B2(n18), .ZN(n13) );
  OAI21_X1 U35 ( .B1(n87), .B2(n92), .A(n13), .ZN(n48) );
  NOR2_X1 U36 ( .A1(n15), .A2(n14), .ZN(n27) );
  OR2_X1 U37 ( .A1(n16), .A2(n27), .ZN(n23) );
  FA_X1 U38 ( .A(bfp_exponent[1]), .B(n18), .CI(n17), .CO(n25), .S(n12) );
  NAND2_X1 U39 ( .A1(n89), .A2(n23), .ZN(n24) );
  NOR2_X1 U40 ( .A1(n89), .A2(n23), .ZN(n26) );
  INV_X1 U41 ( .A(n26), .ZN(n19) );
  NAND2_X1 U42 ( .A1(n24), .A2(n19), .ZN(n20) );
  XNOR2_X1 U43 ( .A(n25), .B(n20), .ZN(n21) );
  AOI22_X1 U44 ( .A1(n56), .A2(bfp_exponent[2]), .B1(n83), .B2(n21), .ZN(n22)
         );
  OAI21_X1 U45 ( .B1(n23), .B2(n58), .A(n22), .ZN(n47) );
  OAI21_X1 U46 ( .B1(n26), .B2(n25), .A(n24), .ZN(n51) );
  INV_X1 U47 ( .A(n27), .ZN(n35) );
  XOR2_X1 U48 ( .A(n34), .B(n35), .Z(n30) );
  NOR2_X1 U49 ( .A1(bfp_exponent[3]), .A2(n30), .ZN(n50) );
  NAND2_X1 U50 ( .A1(bfp_exponent[3]), .A2(n30), .ZN(n52) );
  INV_X1 U51 ( .A(n52), .ZN(n28) );
  NOR2_X1 U52 ( .A1(n50), .A2(n28), .ZN(n29) );
  XOR2_X1 U53 ( .A(n51), .B(n29), .Z(n32) );
  AOI22_X1 U54 ( .A1(n56), .A2(bfp_exponent[3]), .B1(n30), .B2(n65), .ZN(n31)
         );
  OAI21_X1 U55 ( .B1(n33), .B2(n32), .A(n31), .ZN(n46) );
  NOR2_X1 U56 ( .A1(n35), .A2(n34), .ZN(n61) );
  NAND2_X1 U57 ( .A1(n61), .A2(n60), .ZN(n36) );
  OAI211_X1 U58 ( .C1(n61), .C2(n60), .A(n64), .B(n36), .ZN(n59) );
  AOI21_X1 U59 ( .B1(n52), .B2(n51), .A(n50), .ZN(n67) );
  NAND2_X1 U60 ( .A1(n90), .A2(n59), .ZN(n66) );
  NOR2_X1 U61 ( .A1(n90), .A2(n59), .ZN(n68) );
  INV_X1 U62 ( .A(n68), .ZN(n53) );
  NAND2_X1 U63 ( .A1(n66), .A2(n53), .ZN(n54) );
  XNOR2_X1 U64 ( .A(n67), .B(n54), .ZN(n55) );
  AOI22_X1 U65 ( .A1(n56), .A2(bfp_exponent[4]), .B1(n83), .B2(n55), .ZN(n57)
         );
  OAI21_X1 U66 ( .B1(n59), .B2(n58), .A(n57), .ZN(n45) );
  INV_X1 U67 ( .A(n60), .ZN(n62) );
  NOR2_X1 U68 ( .A1(n62), .A2(n61), .ZN(n63) );
  NAND2_X1 U69 ( .A1(n64), .A2(n63), .ZN(n81) );
  INV_X1 U70 ( .A(n81), .ZN(n73) );
  NAND2_X1 U71 ( .A1(n73), .A2(n65), .ZN(n85) );
  OAI21_X1 U72 ( .B1(n68), .B2(n67), .A(n66), .ZN(n77) );
  NAND2_X1 U73 ( .A1(n73), .A2(bfp_exponent[5]), .ZN(n78) );
  OAI21_X1 U74 ( .B1(n73), .B2(bfp_exponent[5]), .A(n78), .ZN(n70) );
  NAND2_X1 U75 ( .A1(n77), .A2(n70), .ZN(n69) );
  OAI211_X1 U76 ( .C1(n77), .C2(n70), .A(n83), .B(n69), .ZN(n71) );
  OAI211_X1 U77 ( .C1(n91), .C2(n87), .A(n85), .B(n71), .ZN(n44) );
  INV_X1 U78 ( .A(n77), .ZN(n72) );
  OAI33_X1 U79 ( .A1(n73), .A2(n91), .A3(n77), .B1(n81), .B2(bfp_exponent[5]), 
        .B3(n72), .ZN(n75) );
  NAND2_X1 U80 ( .A1(bfp_exponent[6]), .A2(n75), .ZN(n74) );
  OAI211_X1 U81 ( .C1(bfp_exponent[6]), .C2(n75), .A(n83), .B(n74), .ZN(n76)
         );
  OAI211_X1 U82 ( .C1(n88), .C2(n87), .A(n85), .B(n76), .ZN(n43) );
  NAND2_X1 U83 ( .A1(n78), .A2(n77), .ZN(n80) );
  NAND2_X1 U84 ( .A1(n80), .A2(n81), .ZN(n79) );
  OAI33_X1 U85 ( .A1(bfp_exponent[6]), .A2(n81), .A3(n80), .B1(n91), .B2(n88), 
        .B3(n79), .ZN(n84) );
  NAND2_X1 U86 ( .A1(bfp_exponent[7]), .A2(n84), .ZN(n82) );
  OAI211_X1 U87 ( .C1(bfp_exponent[7]), .C2(n84), .A(n83), .B(n82), .ZN(n86)
         );
  OAI211_X1 U88 ( .C1(n93), .C2(n87), .A(n86), .B(n85), .ZN(n42) );
endmodule


module fftAddressGenerator_FFT_N10_STAGE_COUNT_BW4 ( clk, rst, stageCount, run, 
        done, act, ctrl, evenOdd, MemAddr, twiddleFactorAddr );
  input [3:0] stageCount;
  output [1:0] ctrl;
  output [8:0] MemAddr;
  output [8:0] twiddleFactorAddr;
  input clk, rst, run;
  output done, act, evenOdd;
  wire   N20, n83, n84, n85, n86, n87, n88, n89, n90, n91, n1, n2, n3, n4, n5,
         n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19, n20,
         n21, n22, n23, n24, n25, n26, n27, n28, n29, n30, n31, n32, n33, n34,
         n35, n36, n37, n38, n39, n40, n41, n42, n43, n44, n45, n46, n47, n48,
         n49, n50, n51, n52, n53, n54, n55, n56, n57, n58, n59, n60, n61, n62,
         n63, n64, n65, n66, n67, n68, n69, n70, n71, n72, n73, n74, n75, n76,
         n77, n78, n79, n80, n81, n82, n92, n93, n94, n95, n96, n97, n98, n99,
         n100, n101, n102, n103;
  wire   [8:1] runCount;
  tri   clk;
  tri   rst;
  tri   act;
  tri   evenOdd;
  tri   [8:0] twiddleFactorAddr;

  DFF_X1 \runCount_reg[0]  ( .D(n91), .CK(clk), .Q(evenOdd) );
  DFF_X1 \runCount_reg[8]  ( .D(n90), .CK(clk), .Q(runCount[8]), .QN(n99) );
  DFF_X1 \runCount_reg[7]  ( .D(n89), .CK(clk), .Q(runCount[7]), .QN(n103) );
  DFF_X1 \runCount_reg[6]  ( .D(n88), .CK(clk), .Q(runCount[6]), .QN(n97) );
  DFF_X1 \runCount_reg[5]  ( .D(n87), .CK(clk), .Q(runCount[5]), .QN(n100) );
  DFF_X1 \runCount_reg[4]  ( .D(n86), .CK(clk), .Q(runCount[4]), .QN(n102) );
  DFF_X1 \runCount_reg[3]  ( .D(n85), .CK(clk), .Q(runCount[3]) );
  DFF_X1 \runCount_reg[2]  ( .D(n84), .CK(clk), .Q(runCount[2]), .QN(n101) );
  DFF_X1 \runCount_reg[1]  ( .D(n83), .CK(clk), .Q(runCount[1]), .QN(n98) );
  DFF_X1 runCount_full_f_reg ( .D(N20), .CK(clk), .Q(done) );
  INV_X1 U3 ( .A(stageCount[1]), .ZN(n53) );
  NOR2_X1 U4 ( .A1(stageCount[3]), .A2(stageCount[2]), .ZN(n51) );
  INV_X1 U5 ( .A(run), .ZN(n29) );
  NOR2_X1 U6 ( .A1(done), .A2(n29), .ZN(act) );
  NAND2_X1 U7 ( .A1(n51), .A2(n53), .ZN(n70) );
  INV_X1 U8 ( .A(n70), .ZN(n26) );
  INV_X1 U9 ( .A(evenOdd), .ZN(n27) );
  AOI22_X1 U10 ( .A1(n26), .A2(n27), .B1(n98), .B2(n70), .ZN(MemAddr[0]) );
  NOR2_X1 U11 ( .A1(stageCount[1]), .A2(stageCount[0]), .ZN(n57) );
  INV_X1 U12 ( .A(n57), .ZN(n80) );
  INV_X1 U13 ( .A(stageCount[2]), .ZN(n93) );
  NAND2_X1 U14 ( .A1(n57), .A2(n93), .ZN(n96) );
  INV_X1 U15 ( .A(n96), .ZN(n18) );
  AOI21_X1 U16 ( .B1(stageCount[2]), .B2(n80), .A(n18), .ZN(n23) );
  INV_X1 U17 ( .A(stageCount[3]), .ZN(n13) );
  NOR2_X1 U18 ( .A1(n18), .A2(n13), .ZN(n25) );
  INV_X1 U19 ( .A(n25), .ZN(n3) );
  NAND3_X1 U20 ( .A1(n23), .A2(evenOdd), .A3(n3), .ZN(n6) );
  INV_X1 U21 ( .A(stageCount[0]), .ZN(n65) );
  NAND2_X1 U22 ( .A1(stageCount[1]), .A2(n65), .ZN(n56) );
  INV_X1 U23 ( .A(n51), .ZN(n61) );
  NOR2_X1 U24 ( .A1(n56), .A2(n61), .ZN(n71) );
  OR2_X1 U25 ( .A1(n26), .A2(n71), .ZN(n1) );
  OAI222_X1 U26 ( .A1(n6), .A2(n56), .B1(n1), .B2(n101), .C1(n70), .C2(n98), 
        .ZN(MemAddr[1]) );
  NAND2_X1 U27 ( .A1(stageCount[1]), .A2(stageCount[0]), .ZN(n60) );
  AOI22_X1 U28 ( .A1(runCount[3]), .A2(n61), .B1(runCount[2]), .B2(n1), .ZN(n2) );
  OAI21_X1 U29 ( .B1(n60), .B2(n6), .A(n2), .ZN(MemAddr[2]) );
  AOI21_X1 U30 ( .B1(n23), .B2(n3), .A(n102), .ZN(n4) );
  AOI22_X1 U31 ( .A1(runCount[3]), .A2(n51), .B1(n4), .B2(n70), .ZN(n5) );
  OAI21_X1 U32 ( .B1(n80), .B2(n6), .A(n5), .ZN(MemAddr[3]) );
  AOI21_X1 U33 ( .B1(stageCount[1]), .B2(stageCount[2]), .A(stageCount[3]), 
        .ZN(n14) );
  AOI21_X1 U34 ( .B1(stageCount[2]), .B2(n80), .A(stageCount[3]), .ZN(n7) );
  NOR2_X1 U35 ( .A1(stageCount[3]), .A2(n96), .ZN(n73) );
  NOR3_X1 U36 ( .A1(n73), .A2(n23), .A3(n25), .ZN(n9) );
  NAND2_X1 U37 ( .A1(stageCount[0]), .A2(n53), .ZN(n79) );
  NOR2_X1 U38 ( .A1(n27), .A2(n79), .ZN(n22) );
  AOI22_X1 U39 ( .A1(runCount[4]), .A2(n7), .B1(n9), .B2(n22), .ZN(n8) );
  OAI21_X1 U40 ( .B1(n100), .B2(n14), .A(n8), .ZN(MemAddr[4]) );
  NAND2_X1 U41 ( .A1(evenOdd), .A2(n9), .ZN(n21) );
  NAND3_X1 U42 ( .A1(stageCount[1]), .A2(stageCount[2]), .A3(stageCount[0]), 
        .ZN(n10) );
  NAND2_X1 U43 ( .A1(n10), .A2(n13), .ZN(n11) );
  AOI22_X1 U44 ( .A1(runCount[6]), .A2(n11), .B1(runCount[5]), .B2(n14), .ZN(
        n12) );
  OAI21_X1 U45 ( .B1(n56), .B2(n21), .A(n12), .ZN(MemAddr[5]) );
  NAND2_X1 U46 ( .A1(runCount[7]), .A2(stageCount[3]), .ZN(n17) );
  NAND2_X1 U47 ( .A1(stageCount[2]), .A2(n13), .ZN(n78) );
  NOR2_X1 U48 ( .A1(n56), .A2(n78), .ZN(n15) );
  OAI21_X1 U49 ( .B1(n15), .B2(n14), .A(runCount[6]), .ZN(n16) );
  OAI211_X1 U50 ( .C1(n60), .C2(n21), .A(n17), .B(n16), .ZN(MemAddr[6]) );
  OAI21_X1 U51 ( .B1(n99), .B2(n18), .A(stageCount[3]), .ZN(n19) );
  OAI21_X1 U52 ( .B1(stageCount[3]), .B2(runCount[7]), .A(n19), .ZN(n20) );
  OAI21_X1 U53 ( .B1(n80), .B2(n21), .A(n20), .ZN(MemAddr[7]) );
  NAND3_X1 U54 ( .A1(n23), .A2(n22), .A3(n70), .ZN(n24) );
  OAI21_X1 U55 ( .B1(n25), .B2(n99), .A(n24), .ZN(MemAddr[8]) );
  AOI21_X1 U56 ( .B1(n26), .B2(n65), .A(n27), .ZN(ctrl[0]) );
  OAI21_X1 U57 ( .B1(stageCount[0]), .B2(n70), .A(n27), .ZN(ctrl[1]) );
  NOR2_X1 U58 ( .A1(n98), .A2(n27), .ZN(n44) );
  INV_X1 U59 ( .A(n44), .ZN(n47) );
  NOR2_X1 U60 ( .A1(n101), .A2(n47), .ZN(n46) );
  NAND2_X1 U61 ( .A1(runCount[3]), .A2(n46), .ZN(n42) );
  NOR2_X1 U62 ( .A1(n102), .A2(n42), .ZN(n41) );
  NAND2_X1 U63 ( .A1(runCount[5]), .A2(n41), .ZN(n37) );
  NOR2_X1 U64 ( .A1(n97), .A2(n37), .ZN(n36) );
  NAND2_X1 U65 ( .A1(runCount[7]), .A2(n36), .ZN(n32) );
  NOR2_X1 U66 ( .A1(n99), .A2(n32), .ZN(n30) );
  INV_X1 U67 ( .A(n30), .ZN(n28) );
  NOR2_X1 U68 ( .A1(n28), .A2(rst), .ZN(N20) );
  NOR2_X1 U69 ( .A1(n29), .A2(rst), .ZN(n48) );
  INV_X1 U70 ( .A(n48), .ZN(n31) );
  NAND2_X1 U71 ( .A1(n48), .A2(n30), .ZN(n50) );
  OAI21_X1 U72 ( .B1(evenOdd), .B2(n31), .A(n50), .ZN(n91) );
  AOI21_X1 U73 ( .B1(n99), .B2(n32), .A(n31), .ZN(n90) );
  OAI211_X1 U74 ( .C1(runCount[7]), .C2(n36), .A(n48), .B(n32), .ZN(n33) );
  NAND2_X1 U75 ( .A1(n50), .A2(n33), .ZN(n89) );
  INV_X1 U76 ( .A(n37), .ZN(n34) );
  OAI21_X1 U77 ( .B1(runCount[6]), .B2(n34), .A(n48), .ZN(n35) );
  OAI21_X1 U78 ( .B1(n36), .B2(n35), .A(n50), .ZN(n88) );
  OAI211_X1 U79 ( .C1(runCount[5]), .C2(n41), .A(n48), .B(n37), .ZN(n38) );
  NAND2_X1 U80 ( .A1(n50), .A2(n38), .ZN(n87) );
  INV_X1 U81 ( .A(n42), .ZN(n39) );
  OAI21_X1 U82 ( .B1(runCount[4]), .B2(n39), .A(n48), .ZN(n40) );
  OAI21_X1 U83 ( .B1(n41), .B2(n40), .A(n50), .ZN(n86) );
  OAI211_X1 U84 ( .C1(runCount[3]), .C2(n46), .A(n48), .B(n42), .ZN(n43) );
  NAND2_X1 U85 ( .A1(n50), .A2(n43), .ZN(n85) );
  OAI21_X1 U86 ( .B1(runCount[2]), .B2(n44), .A(n48), .ZN(n45) );
  OAI21_X1 U87 ( .B1(n46), .B2(n45), .A(n50), .ZN(n84) );
  OAI211_X1 U88 ( .C1(runCount[1]), .C2(evenOdd), .A(n48), .B(n47), .ZN(n49)
         );
  NAND2_X1 U89 ( .A1(n50), .A2(n49), .ZN(n83) );
  AND2_X1 U90 ( .A1(runCount[8]), .A2(n73), .ZN(twiddleFactorAddr[0]) );
  AOI22_X1 U91 ( .A1(stageCount[0]), .A2(n99), .B1(n103), .B2(n65), .ZN(n52)
         );
  NAND2_X1 U92 ( .A1(n53), .A2(n52), .ZN(n64) );
  NOR2_X1 U93 ( .A1(n61), .A2(n64), .ZN(twiddleFactorAddr[1]) );
  OAI222_X1 U94 ( .A1(n80), .A2(n97), .B1(n56), .B2(n99), .C1(n103), .C2(n79), 
        .ZN(n66) );
  AND2_X1 U95 ( .A1(n51), .A2(n66), .ZN(twiddleFactorAddr[2]) );
  OAI222_X1 U96 ( .A1(n53), .A2(n52), .B1(n80), .B2(runCount[5]), .C1(n79), 
        .C2(runCount[6]), .ZN(n77) );
  NOR2_X1 U97 ( .A1(n61), .A2(n77), .ZN(twiddleFactorAddr[3]) );
  OAI22_X1 U98 ( .A1(n103), .A2(n60), .B1(n100), .B2(n79), .ZN(n54) );
  AOI21_X1 U99 ( .B1(runCount[4]), .B2(n57), .A(n54), .ZN(n55) );
  OAI21_X1 U100 ( .B1(n97), .B2(n56), .A(n55), .ZN(n92) );
  INV_X1 U101 ( .A(n92), .ZN(n59) );
  INV_X1 U102 ( .A(n78), .ZN(n67) );
  NAND2_X1 U103 ( .A1(n57), .A2(n67), .ZN(n58) );
  OAI22_X1 U104 ( .A1(n59), .A2(n61), .B1(n99), .B2(n58), .ZN(
        twiddleFactorAddr[4]) );
  NOR2_X1 U105 ( .A1(n61), .A2(n60), .ZN(n72) );
  AOI22_X1 U106 ( .A1(runCount[6]), .A2(n72), .B1(runCount[5]), .B2(n71), .ZN(
        n63) );
  NOR2_X1 U107 ( .A1(n65), .A2(n70), .ZN(n74) );
  AOI22_X1 U108 ( .A1(runCount[4]), .A2(n74), .B1(runCount[3]), .B2(n73), .ZN(
        n62) );
  OAI211_X1 U109 ( .C1(n78), .C2(n64), .A(n63), .B(n62), .ZN(
        twiddleFactorAddr[5]) );
  AOI22_X1 U110 ( .A1(stageCount[0]), .A2(runCount[3]), .B1(runCount[2]), .B2(
        n65), .ZN(n82) );
  AOI22_X1 U111 ( .A1(runCount[5]), .A2(n72), .B1(runCount[4]), .B2(n71), .ZN(
        n69) );
  NAND2_X1 U112 ( .A1(n67), .A2(n66), .ZN(n68) );
  OAI211_X1 U113 ( .C1(n82), .C2(n70), .A(n69), .B(n68), .ZN(
        twiddleFactorAddr[6]) );
  AOI22_X1 U114 ( .A1(runCount[4]), .A2(n72), .B1(runCount[3]), .B2(n71), .ZN(
        n76) );
  AOI22_X1 U115 ( .A1(runCount[2]), .A2(n74), .B1(runCount[1]), .B2(n73), .ZN(
        n75) );
  OAI211_X1 U116 ( .C1(n78), .C2(n77), .A(n76), .B(n75), .ZN(
        twiddleFactorAddr[7]) );
  OAI22_X1 U117 ( .A1(evenOdd), .A2(n80), .B1(runCount[1]), .B2(n79), .ZN(n81)
         );
  AOI21_X1 U118 ( .B1(stageCount[1]), .B2(n82), .A(n81), .ZN(n94) );
  AOI221_X1 U119 ( .B1(n94), .B2(n93), .C1(n92), .C2(stageCount[2]), .A(
        stageCount[3]), .ZN(n95) );
  AOI221_X1 U120 ( .B1(n99), .B2(stageCount[3]), .C1(n96), .C2(stageCount[3]), 
        .A(n95), .ZN(twiddleFactorAddr[8]) );
endmodule


module bfp_Shifter_FFT_DW16_FFT_BFPDW5_3 ( operand, bfp_operand, bw );
  input [15:0] operand;
  output [15:0] bfp_operand;
  input [4:0] bw;
  wire   N8, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15,
         n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29,
         n30, n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43,
         n44, n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57,
         n58, n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71,
         n72, n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85,
         n86, n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99,
         n100;
  tri   [15:0] operand;
  assign N8 = bw[4];

  INV_X1 U2 ( .A(n95), .ZN(n92) );
  NOR2_X2 U3 ( .A1(bw[0]), .A2(bw[1]), .ZN(n73) );
  AOI22_X2 U4 ( .A1(bw[3]), .A2(n4), .B1(n73), .B2(n30), .ZN(n84) );
  CLKBUF_X1 U5 ( .A(bw[2]), .Z(n95) );
  INV_X1 U6 ( .A(n73), .ZN(n89) );
  INV_X1 U7 ( .A(bw[1]), .ZN(n13) );
  NOR2_X1 U8 ( .A1(bw[0]), .A2(n13), .ZN(n65) );
  INV_X1 U9 ( .A(operand[4]), .ZN(n7) );
  NAND2_X1 U10 ( .A1(bw[0]), .A2(bw[1]), .ZN(n27) );
  INV_X1 U11 ( .A(bw[0]), .ZN(n3) );
  NOR2_X1 U12 ( .A1(bw[1]), .A2(n3), .ZN(n72) );
  INV_X1 U13 ( .A(n72), .ZN(n90) );
  OAI22_X1 U14 ( .A1(operand[5]), .A2(n27), .B1(operand[3]), .B2(n90), .ZN(n1)
         );
  AOI21_X1 U15 ( .B1(n65), .B2(n7), .A(n1), .ZN(n2) );
  OAI21_X1 U16 ( .B1(operand[2]), .B2(n89), .A(n2), .ZN(n38) );
  INV_X1 U17 ( .A(operand[1]), .ZN(n17) );
  INV_X1 U18 ( .A(operand[0]), .ZN(n16) );
  AOI22_X1 U19 ( .A1(bw[0]), .A2(n17), .B1(n16), .B2(n3), .ZN(n14) );
  NAND2_X1 U20 ( .A1(bw[1]), .A2(n14), .ZN(n34) );
  AOI22_X1 U21 ( .A1(n95), .A2(n38), .B1(n34), .B2(n92), .ZN(n70) );
  INV_X1 U22 ( .A(bw[3]), .ZN(n76) );
  NOR3_X1 U23 ( .A1(N8), .A2(n92), .A3(n27), .ZN(n4) );
  NOR2_X1 U24 ( .A1(n95), .A2(bw[3]), .ZN(n30) );
  INV_X1 U25 ( .A(n84), .ZN(n96) );
  NOR2_X1 U26 ( .A1(N8), .A2(n96), .ZN(n88) );
  INV_X1 U27 ( .A(n88), .ZN(n82) );
  NOR2_X1 U28 ( .A1(n76), .A2(n82), .ZN(n97) );
  AOI22_X1 U29 ( .A1(n70), .A2(n97), .B1(n96), .B2(operand[5]), .ZN(n5) );
  INV_X1 U30 ( .A(n5), .ZN(bfp_operand[5]) );
  AOI22_X1 U31 ( .A1(n65), .A2(operand[3]), .B1(n72), .B2(operand[2]), .ZN(n6)
         );
  OAI21_X1 U32 ( .B1(n27), .B2(n7), .A(n6), .ZN(n8) );
  AOI21_X1 U33 ( .B1(n73), .B2(operand[1]), .A(n8), .ZN(n29) );
  NOR2_X1 U34 ( .A1(n27), .A2(n16), .ZN(n32) );
  INV_X1 U35 ( .A(n32), .ZN(n9) );
  AOI22_X1 U36 ( .A1(n95), .A2(n29), .B1(n9), .B2(n92), .ZN(n62) );
  AOI22_X1 U37 ( .A1(n62), .A2(n97), .B1(n96), .B2(operand[4]), .ZN(n10) );
  INV_X1 U38 ( .A(n10), .ZN(bfp_operand[4]) );
  INV_X1 U39 ( .A(n65), .ZN(n91) );
  INV_X1 U40 ( .A(operand[6]), .ZN(n25) );
  INV_X1 U41 ( .A(n27), .ZN(n48) );
  AOI22_X1 U42 ( .A1(n48), .A2(operand[7]), .B1(n72), .B2(operand[5]), .ZN(n11) );
  OAI21_X1 U43 ( .B1(n91), .B2(n25), .A(n11), .ZN(n12) );
  AOI21_X1 U44 ( .B1(n73), .B2(operand[4]), .A(n12), .ZN(n52) );
  AOI222_X1 U45 ( .A1(n14), .A2(n13), .B1(n48), .B2(operand[3]), .C1(
        operand[2]), .C2(n65), .ZN(n47) );
  AOI22_X1 U46 ( .A1(n95), .A2(n52), .B1(n47), .B2(n92), .ZN(n87) );
  AOI22_X1 U47 ( .A1(n96), .A2(operand[7]), .B1(n87), .B2(n97), .ZN(n15) );
  INV_X1 U48 ( .A(n15), .ZN(bfp_operand[7]) );
  NOR2_X1 U49 ( .A1(n84), .A2(n16), .ZN(bfp_operand[0]) );
  NAND2_X1 U50 ( .A1(n95), .A2(n97), .ZN(n19) );
  OAI22_X1 U51 ( .A1(n84), .A2(n17), .B1(n19), .B2(n34), .ZN(bfp_operand[1])
         );
  INV_X1 U52 ( .A(operand[2]), .ZN(n18) );
  AOI222_X1 U53 ( .A1(n48), .A2(operand[2]), .B1(n65), .B2(operand[1]), .C1(
        n72), .C2(operand[0]), .ZN(n21) );
  OAI22_X1 U54 ( .A1(n84), .A2(n18), .B1(n21), .B2(n19), .ZN(bfp_operand[2])
         );
  INV_X1 U55 ( .A(operand[3]), .ZN(n20) );
  OAI22_X1 U56 ( .A1(n84), .A2(n20), .B1(n47), .B2(n19), .ZN(bfp_operand[3])
         );
  INV_X1 U57 ( .A(n21), .ZN(n80) );
  AOI22_X1 U58 ( .A1(n48), .A2(operand[6]), .B1(n65), .B2(operand[5]), .ZN(n23) );
  AOI22_X1 U59 ( .A1(n73), .A2(operand[3]), .B1(n72), .B2(operand[4]), .ZN(n22) );
  NAND2_X1 U60 ( .A1(n23), .A2(n22), .ZN(n79) );
  OAI221_X1 U61 ( .B1(n95), .B2(n80), .C1(n92), .C2(n79), .A(n97), .ZN(n24) );
  OAI21_X1 U62 ( .B1(n84), .B2(n25), .A(n24), .ZN(bfp_operand[6]) );
  INV_X1 U63 ( .A(operand[8]), .ZN(n36) );
  AOI22_X1 U64 ( .A1(n65), .A2(operand[7]), .B1(n72), .B2(operand[6]), .ZN(n26) );
  OAI21_X1 U65 ( .B1(n27), .B2(n36), .A(n26), .ZN(n28) );
  AOI21_X1 U66 ( .B1(n73), .B2(operand[5]), .A(n28), .ZN(n58) );
  AOI22_X1 U67 ( .A1(n95), .A2(n58), .B1(n29), .B2(n92), .ZN(n31) );
  NOR2_X1 U68 ( .A1(n30), .A2(n82), .ZN(n53) );
  OAI221_X1 U69 ( .B1(bw[3]), .B2(n32), .C1(n76), .C2(n31), .A(n53), .ZN(n33)
         );
  OAI21_X1 U70 ( .B1(n84), .B2(n36), .A(n33), .ZN(bfp_operand[8]) );
  INV_X1 U71 ( .A(operand[9]), .ZN(n50) );
  INV_X1 U72 ( .A(n34), .ZN(n40) );
  AOI22_X1 U73 ( .A1(n48), .A2(operand[9]), .B1(n72), .B2(operand[7]), .ZN(n35) );
  OAI21_X1 U74 ( .B1(n91), .B2(n36), .A(n35), .ZN(n37) );
  AOI21_X1 U75 ( .B1(n73), .B2(operand[6]), .A(n37), .ZN(n66) );
  AOI22_X1 U76 ( .A1(n95), .A2(n66), .B1(n38), .B2(n92), .ZN(n39) );
  OAI221_X1 U77 ( .B1(bw[3]), .B2(n40), .C1(n76), .C2(n39), .A(n53), .ZN(n41)
         );
  OAI21_X1 U78 ( .B1(n84), .B2(n50), .A(n41), .ZN(bfp_operand[9]) );
  AOI22_X1 U79 ( .A1(n48), .A2(operand[10]), .B1(n65), .B2(operand[9]), .ZN(
        n43) );
  AOI22_X1 U80 ( .A1(n73), .A2(operand[7]), .B1(n72), .B2(operand[8]), .ZN(n42) );
  NAND2_X1 U81 ( .A1(n43), .A2(n42), .ZN(n77) );
  AOI221_X1 U82 ( .B1(n95), .B2(n77), .C1(n92), .C2(n79), .A(n76), .ZN(n46) );
  OAI21_X1 U83 ( .B1(bw[3]), .B2(n80), .A(n53), .ZN(n45) );
  INV_X1 U84 ( .A(operand[10]), .ZN(n44) );
  OAI22_X1 U85 ( .A1(n46), .A2(n45), .B1(n84), .B2(n44), .ZN(bfp_operand[10])
         );
  INV_X1 U86 ( .A(operand[11]), .ZN(n57) );
  INV_X1 U87 ( .A(n47), .ZN(n55) );
  AOI22_X1 U88 ( .A1(n48), .A2(operand[11]), .B1(n65), .B2(operand[10]), .ZN(
        n49) );
  OAI21_X1 U89 ( .B1(n50), .B2(n90), .A(n49), .ZN(n51) );
  AOI21_X1 U90 ( .B1(n73), .B2(operand[8]), .A(n51), .ZN(n93) );
  AOI22_X1 U91 ( .A1(n95), .A2(n93), .B1(n52), .B2(n92), .ZN(n54) );
  OAI221_X1 U92 ( .B1(bw[3]), .B2(n55), .C1(n76), .C2(n54), .A(n53), .ZN(n56)
         );
  OAI21_X1 U93 ( .B1(n84), .B2(n57), .A(n56), .ZN(bfp_operand[11]) );
  INV_X1 U94 ( .A(operand[12]), .ZN(n64) );
  AOI21_X1 U95 ( .B1(n65), .B2(operand[11]), .A(n92), .ZN(n60) );
  AOI22_X1 U96 ( .A1(n73), .A2(operand[9]), .B1(n72), .B2(operand[10]), .ZN(
        n59) );
  AOI22_X1 U97 ( .A1(n60), .A2(n59), .B1(n58), .B2(n92), .ZN(n61) );
  OAI221_X1 U98 ( .B1(bw[3]), .B2(n62), .C1(n76), .C2(n61), .A(n88), .ZN(n63)
         );
  OAI21_X1 U99 ( .B1(n84), .B2(n64), .A(n63), .ZN(bfp_operand[12]) );
  INV_X1 U100 ( .A(operand[13]), .ZN(n75) );
  AOI21_X1 U101 ( .B1(n65), .B2(operand[12]), .A(n92), .ZN(n68) );
  AOI22_X1 U102 ( .A1(n73), .A2(operand[10]), .B1(operand[11]), .B2(n72), .ZN(
        n67) );
  AOI22_X1 U103 ( .A1(n68), .A2(n67), .B1(n66), .B2(n92), .ZN(n69) );
  OAI221_X1 U104 ( .B1(bw[3]), .B2(n70), .C1(n76), .C2(n69), .A(n88), .ZN(n71)
         );
  OAI21_X1 U105 ( .B1(n84), .B2(n75), .A(n71), .ZN(bfp_operand[13]) );
  AOI22_X1 U106 ( .A1(n73), .A2(operand[11]), .B1(n72), .B2(operand[12]), .ZN(
        n74) );
  OAI21_X1 U107 ( .B1(n91), .B2(n75), .A(n74), .ZN(n78) );
  AOI221_X1 U108 ( .B1(n95), .B2(n78), .C1(n92), .C2(n77), .A(n76), .ZN(n86)
         );
  AOI221_X1 U109 ( .B1(n92), .B2(n80), .C1(n95), .C2(n79), .A(bw[3]), .ZN(n81)
         );
  OR2_X1 U110 ( .A1(n82), .A2(n81), .ZN(n85) );
  INV_X1 U111 ( .A(operand[14]), .ZN(n83) );
  OAI22_X1 U112 ( .A1(n86), .A2(n85), .B1(n84), .B2(n83), .ZN(bfp_operand[14])
         );
  NAND2_X1 U113 ( .A1(n88), .A2(n87), .ZN(n100) );
  OAI222_X1 U114 ( .A1(n91), .A2(operand[14]), .B1(n90), .B2(operand[13]), 
        .C1(n89), .C2(operand[12]), .ZN(n94) );
  AOI22_X1 U115 ( .A1(n95), .A2(n94), .B1(n93), .B2(n92), .ZN(n98) );
  AOI22_X1 U116 ( .A1(n98), .A2(n97), .B1(operand[15]), .B2(n96), .ZN(n99) );
  OAI21_X1 U117 ( .B1(bw[3]), .B2(n100), .A(n99), .ZN(bfp_operand[15]) );
endmodule


module ramPipelineBridge_FFT_N10_FFT_DW16_1 ( clk, rst, iact, oact, ictrl, 
        octrl, iMemAddr, iEvenData, iOddData, oMemAddr, oEvenData, oOddData );
  input [1:0] ictrl;
  output [1:0] octrl;
  input [8:0] iMemAddr;
  input [31:0] iEvenData;
  input [31:0] iOddData;
  output [8:0] oMemAddr;
  output [31:0] oEvenData;
  output [31:0] oOddData;
  input clk, rst, iact;
  output oact;
  wire   actPipe, N7, N8, N9, N10, N11, N12, n1, n2, n3;
  wire   [8:0] memPipeAddr;
  wire   [31:0] evenPipeData;
  wire   [31:0] oddPipeData;
  wire   [1:0] ctrlPipe;
  wire   [31:0] ev0Data;
  wire   [31:0] od0Data;
  wire   [31:0] od1Data;
  tri   clk;
  tri   rst;

  DFF_X1 \memPipeAddr_reg[8]  ( .D(iMemAddr[8]), .CK(clk), .Q(memPipeAddr[8])
         );
  DFF_X1 \memPipeAddr_reg[7]  ( .D(iMemAddr[7]), .CK(clk), .Q(memPipeAddr[7])
         );
  DFF_X1 \memPipeAddr_reg[6]  ( .D(iMemAddr[6]), .CK(clk), .Q(memPipeAddr[6])
         );
  DFF_X1 \memPipeAddr_reg[5]  ( .D(iMemAddr[5]), .CK(clk), .Q(memPipeAddr[5])
         );
  DFF_X1 \memPipeAddr_reg[4]  ( .D(iMemAddr[4]), .CK(clk), .Q(memPipeAddr[4])
         );
  DFF_X1 \memPipeAddr_reg[3]  ( .D(iMemAddr[3]), .CK(clk), .Q(memPipeAddr[3])
         );
  DFF_X1 \memPipeAddr_reg[2]  ( .D(iMemAddr[2]), .CK(clk), .Q(memPipeAddr[2])
         );
  DFF_X1 \memPipeAddr_reg[1]  ( .D(iMemAddr[1]), .CK(clk), .Q(memPipeAddr[1])
         );
  DFF_X1 \memPipeAddr_reg[0]  ( .D(iMemAddr[0]), .CK(clk), .Q(memPipeAddr[0])
         );
  DFF_X1 \evenPipeData_reg[31]  ( .D(iEvenData[31]), .CK(clk), .Q(
        evenPipeData[31]) );
  DFF_X1 \evenPipeData_reg[30]  ( .D(iEvenData[30]), .CK(clk), .Q(
        evenPipeData[30]) );
  DFF_X1 \evenPipeData_reg[29]  ( .D(iEvenData[29]), .CK(clk), .Q(
        evenPipeData[29]) );
  DFF_X1 \evenPipeData_reg[28]  ( .D(iEvenData[28]), .CK(clk), .Q(
        evenPipeData[28]) );
  DFF_X1 \evenPipeData_reg[27]  ( .D(iEvenData[27]), .CK(clk), .Q(
        evenPipeData[27]) );
  DFF_X1 \evenPipeData_reg[26]  ( .D(iEvenData[26]), .CK(clk), .Q(
        evenPipeData[26]) );
  DFF_X1 \evenPipeData_reg[25]  ( .D(iEvenData[25]), .CK(clk), .Q(
        evenPipeData[25]) );
  DFF_X1 \evenPipeData_reg[24]  ( .D(iEvenData[24]), .CK(clk), .Q(
        evenPipeData[24]) );
  DFF_X1 \evenPipeData_reg[23]  ( .D(iEvenData[23]), .CK(clk), .Q(
        evenPipeData[23]) );
  DFF_X1 \evenPipeData_reg[22]  ( .D(iEvenData[22]), .CK(clk), .Q(
        evenPipeData[22]) );
  DFF_X1 \evenPipeData_reg[21]  ( .D(iEvenData[21]), .CK(clk), .Q(
        evenPipeData[21]) );
  DFF_X1 \evenPipeData_reg[20]  ( .D(iEvenData[20]), .CK(clk), .Q(
        evenPipeData[20]) );
  DFF_X1 \evenPipeData_reg[19]  ( .D(iEvenData[19]), .CK(clk), .Q(
        evenPipeData[19]) );
  DFF_X1 \evenPipeData_reg[18]  ( .D(iEvenData[18]), .CK(clk), .Q(
        evenPipeData[18]) );
  DFF_X1 \evenPipeData_reg[17]  ( .D(iEvenData[17]), .CK(clk), .Q(
        evenPipeData[17]) );
  DFF_X1 \evenPipeData_reg[16]  ( .D(iEvenData[16]), .CK(clk), .Q(
        evenPipeData[16]) );
  DFF_X1 \evenPipeData_reg[15]  ( .D(iEvenData[15]), .CK(clk), .Q(
        evenPipeData[15]) );
  DFF_X1 \evenPipeData_reg[14]  ( .D(iEvenData[14]), .CK(clk), .Q(
        evenPipeData[14]) );
  DFF_X1 \evenPipeData_reg[13]  ( .D(iEvenData[13]), .CK(clk), .Q(
        evenPipeData[13]) );
  DFF_X1 \evenPipeData_reg[12]  ( .D(iEvenData[12]), .CK(clk), .Q(
        evenPipeData[12]) );
  DFF_X1 \evenPipeData_reg[11]  ( .D(iEvenData[11]), .CK(clk), .Q(
        evenPipeData[11]) );
  DFF_X1 \evenPipeData_reg[10]  ( .D(iEvenData[10]), .CK(clk), .Q(
        evenPipeData[10]) );
  DFF_X1 \evenPipeData_reg[9]  ( .D(iEvenData[9]), .CK(clk), .Q(
        evenPipeData[9]) );
  DFF_X1 \evenPipeData_reg[8]  ( .D(iEvenData[8]), .CK(clk), .Q(
        evenPipeData[8]) );
  DFF_X1 \evenPipeData_reg[7]  ( .D(iEvenData[7]), .CK(clk), .Q(
        evenPipeData[7]) );
  DFF_X1 \evenPipeData_reg[6]  ( .D(iEvenData[6]), .CK(clk), .Q(
        evenPipeData[6]) );
  DFF_X1 \evenPipeData_reg[5]  ( .D(iEvenData[5]), .CK(clk), .Q(
        evenPipeData[5]) );
  DFF_X1 \evenPipeData_reg[4]  ( .D(iEvenData[4]), .CK(clk), .Q(
        evenPipeData[4]) );
  DFF_X1 \evenPipeData_reg[3]  ( .D(iEvenData[3]), .CK(clk), .Q(
        evenPipeData[3]) );
  DFF_X1 \evenPipeData_reg[2]  ( .D(iEvenData[2]), .CK(clk), .Q(
        evenPipeData[2]) );
  DFF_X1 \evenPipeData_reg[1]  ( .D(iEvenData[1]), .CK(clk), .Q(
        evenPipeData[1]) );
  DFF_X1 \evenPipeData_reg[0]  ( .D(iEvenData[0]), .CK(clk), .Q(
        evenPipeData[0]) );
  DFF_X1 \oddPipeData_reg[31]  ( .D(iOddData[31]), .CK(clk), .Q(
        oddPipeData[31]) );
  DFF_X1 \oddPipeData_reg[30]  ( .D(iOddData[30]), .CK(clk), .Q(
        oddPipeData[30]) );
  DFF_X1 \oddPipeData_reg[29]  ( .D(iOddData[29]), .CK(clk), .Q(
        oddPipeData[29]) );
  DFF_X1 \oddPipeData_reg[28]  ( .D(iOddData[28]), .CK(clk), .Q(
        oddPipeData[28]) );
  DFF_X1 \oddPipeData_reg[27]  ( .D(iOddData[27]), .CK(clk), .Q(
        oddPipeData[27]) );
  DFF_X1 \oddPipeData_reg[26]  ( .D(iOddData[26]), .CK(clk), .Q(
        oddPipeData[26]) );
  DFF_X1 \oddPipeData_reg[25]  ( .D(iOddData[25]), .CK(clk), .Q(
        oddPipeData[25]) );
  DFF_X1 \oddPipeData_reg[24]  ( .D(iOddData[24]), .CK(clk), .Q(
        oddPipeData[24]) );
  DFF_X1 \oddPipeData_reg[23]  ( .D(iOddData[23]), .CK(clk), .Q(
        oddPipeData[23]) );
  DFF_X1 \oddPipeData_reg[22]  ( .D(iOddData[22]), .CK(clk), .Q(
        oddPipeData[22]) );
  DFF_X1 \oddPipeData_reg[21]  ( .D(iOddData[21]), .CK(clk), .Q(
        oddPipeData[21]) );
  DFF_X1 \oddPipeData_reg[20]  ( .D(iOddData[20]), .CK(clk), .Q(
        oddPipeData[20]) );
  DFF_X1 \oddPipeData_reg[19]  ( .D(iOddData[19]), .CK(clk), .Q(
        oddPipeData[19]) );
  DFF_X1 \oddPipeData_reg[18]  ( .D(iOddData[18]), .CK(clk), .Q(
        oddPipeData[18]) );
  DFF_X1 \oddPipeData_reg[17]  ( .D(iOddData[17]), .CK(clk), .Q(
        oddPipeData[17]) );
  DFF_X1 \oddPipeData_reg[16]  ( .D(iOddData[16]), .CK(clk), .Q(
        oddPipeData[16]) );
  DFF_X1 \oddPipeData_reg[15]  ( .D(iOddData[15]), .CK(clk), .Q(
        oddPipeData[15]) );
  DFF_X1 \oddPipeData_reg[14]  ( .D(iOddData[14]), .CK(clk), .Q(
        oddPipeData[14]) );
  DFF_X1 \oddPipeData_reg[13]  ( .D(iOddData[13]), .CK(clk), .Q(
        oddPipeData[13]) );
  DFF_X1 \oddPipeData_reg[12]  ( .D(iOddData[12]), .CK(clk), .Q(
        oddPipeData[12]) );
  DFF_X1 \oddPipeData_reg[11]  ( .D(iOddData[11]), .CK(clk), .Q(
        oddPipeData[11]) );
  DFF_X1 \oddPipeData_reg[10]  ( .D(iOddData[10]), .CK(clk), .Q(
        oddPipeData[10]) );
  DFF_X1 \oddPipeData_reg[9]  ( .D(iOddData[9]), .CK(clk), .Q(oddPipeData[9])
         );
  DFF_X1 \oddPipeData_reg[8]  ( .D(iOddData[8]), .CK(clk), .Q(oddPipeData[8])
         );
  DFF_X1 \oddPipeData_reg[7]  ( .D(iOddData[7]), .CK(clk), .Q(oddPipeData[7])
         );
  DFF_X1 \oddPipeData_reg[6]  ( .D(iOddData[6]), .CK(clk), .Q(oddPipeData[6])
         );
  DFF_X1 \oddPipeData_reg[5]  ( .D(iOddData[5]), .CK(clk), .Q(oddPipeData[5])
         );
  DFF_X1 \oddPipeData_reg[4]  ( .D(iOddData[4]), .CK(clk), .Q(oddPipeData[4])
         );
  DFF_X1 \oddPipeData_reg[3]  ( .D(iOddData[3]), .CK(clk), .Q(oddPipeData[3])
         );
  DFF_X1 \oddPipeData_reg[2]  ( .D(iOddData[2]), .CK(clk), .Q(oddPipeData[2])
         );
  DFF_X1 \oddPipeData_reg[1]  ( .D(iOddData[1]), .CK(clk), .Q(oddPipeData[1])
         );
  DFF_X1 \oddPipeData_reg[0]  ( .D(iOddData[0]), .CK(clk), .Q(oddPipeData[0])
         );
  DFF_X1 \ctrlPipe_reg[1]  ( .D(N9), .CK(clk), .Q(ctrlPipe[1]) );
  DFF_X1 \ctrlPipe_reg[0]  ( .D(N8), .CK(clk), .Q(ctrlPipe[0]) );
  DFF_X1 actPipe_reg ( .D(N7), .CK(clk), .Q(actPipe) );
  DFF_X1 \ctrl_f_reg[1]  ( .D(N12), .CK(clk), .Q(octrl[1]) );
  DFF_X1 \ctrl_f_reg[0]  ( .D(N11), .CK(clk), .Q(octrl[0]) );
  DFF_X1 oact_f_reg ( .D(N10), .CK(clk), .Q(oact) );
  DFF_X1 \mem0Addr_reg[8]  ( .D(memPipeAddr[8]), .CK(clk), .Q(oMemAddr[8]) );
  DFF_X1 \mem0Addr_reg[7]  ( .D(memPipeAddr[7]), .CK(clk), .Q(oMemAddr[7]) );
  DFF_X1 \mem0Addr_reg[6]  ( .D(memPipeAddr[6]), .CK(clk), .Q(oMemAddr[6]) );
  DFF_X1 \mem0Addr_reg[5]  ( .D(memPipeAddr[5]), .CK(clk), .Q(oMemAddr[5]) );
  DFF_X1 \mem0Addr_reg[4]  ( .D(memPipeAddr[4]), .CK(clk), .Q(oMemAddr[4]) );
  DFF_X1 \mem0Addr_reg[3]  ( .D(memPipeAddr[3]), .CK(clk), .Q(oMemAddr[3]) );
  DFF_X1 \mem0Addr_reg[2]  ( .D(memPipeAddr[2]), .CK(clk), .Q(oMemAddr[2]) );
  DFF_X1 \mem0Addr_reg[1]  ( .D(memPipeAddr[1]), .CK(clk), .Q(oMemAddr[1]) );
  DFF_X1 \mem0Addr_reg[0]  ( .D(memPipeAddr[0]), .CK(clk), .Q(oMemAddr[0]) );
  DFF_X1 \ev0Data_reg[31]  ( .D(evenPipeData[31]), .CK(clk), .Q(ev0Data[31])
         );
  DFF_X1 \ev0Data_reg[30]  ( .D(evenPipeData[30]), .CK(clk), .Q(ev0Data[30])
         );
  DFF_X1 \ev0Data_reg[29]  ( .D(evenPipeData[29]), .CK(clk), .Q(ev0Data[29])
         );
  DFF_X1 \ev0Data_reg[28]  ( .D(evenPipeData[28]), .CK(clk), .Q(ev0Data[28])
         );
  DFF_X1 \ev0Data_reg[27]  ( .D(evenPipeData[27]), .CK(clk), .Q(ev0Data[27])
         );
  DFF_X1 \ev0Data_reg[26]  ( .D(evenPipeData[26]), .CK(clk), .Q(ev0Data[26])
         );
  DFF_X1 \ev0Data_reg[25]  ( .D(evenPipeData[25]), .CK(clk), .Q(ev0Data[25])
         );
  DFF_X1 \ev0Data_reg[24]  ( .D(evenPipeData[24]), .CK(clk), .Q(ev0Data[24])
         );
  DFF_X1 \ev0Data_reg[23]  ( .D(evenPipeData[23]), .CK(clk), .Q(ev0Data[23])
         );
  DFF_X1 \ev0Data_reg[22]  ( .D(evenPipeData[22]), .CK(clk), .Q(ev0Data[22])
         );
  DFF_X1 \ev0Data_reg[21]  ( .D(evenPipeData[21]), .CK(clk), .Q(ev0Data[21])
         );
  DFF_X1 \ev0Data_reg[20]  ( .D(evenPipeData[20]), .CK(clk), .Q(ev0Data[20])
         );
  DFF_X1 \ev0Data_reg[19]  ( .D(evenPipeData[19]), .CK(clk), .Q(ev0Data[19])
         );
  DFF_X1 \ev0Data_reg[18]  ( .D(evenPipeData[18]), .CK(clk), .Q(ev0Data[18])
         );
  DFF_X1 \ev0Data_reg[17]  ( .D(evenPipeData[17]), .CK(clk), .Q(ev0Data[17])
         );
  DFF_X1 \ev0Data_reg[16]  ( .D(evenPipeData[16]), .CK(clk), .Q(ev0Data[16])
         );
  DFF_X1 \ev0Data_reg[15]  ( .D(evenPipeData[15]), .CK(clk), .Q(ev0Data[15])
         );
  DFF_X1 \ev0Data_reg[14]  ( .D(evenPipeData[14]), .CK(clk), .Q(ev0Data[14])
         );
  DFF_X1 \ev0Data_reg[13]  ( .D(evenPipeData[13]), .CK(clk), .Q(ev0Data[13])
         );
  DFF_X1 \ev0Data_reg[12]  ( .D(evenPipeData[12]), .CK(clk), .Q(ev0Data[12])
         );
  DFF_X1 \ev0Data_reg[11]  ( .D(evenPipeData[11]), .CK(clk), .Q(ev0Data[11])
         );
  DFF_X1 \ev0Data_reg[10]  ( .D(evenPipeData[10]), .CK(clk), .Q(ev0Data[10])
         );
  DFF_X1 \ev0Data_reg[9]  ( .D(evenPipeData[9]), .CK(clk), .Q(ev0Data[9]) );
  DFF_X1 \ev0Data_reg[8]  ( .D(evenPipeData[8]), .CK(clk), .Q(ev0Data[8]) );
  DFF_X1 \ev0Data_reg[7]  ( .D(evenPipeData[7]), .CK(clk), .Q(ev0Data[7]) );
  DFF_X1 \ev0Data_reg[6]  ( .D(evenPipeData[6]), .CK(clk), .Q(ev0Data[6]) );
  DFF_X1 \ev0Data_reg[5]  ( .D(evenPipeData[5]), .CK(clk), .Q(ev0Data[5]) );
  DFF_X1 \ev0Data_reg[4]  ( .D(evenPipeData[4]), .CK(clk), .Q(ev0Data[4]) );
  DFF_X1 \ev0Data_reg[3]  ( .D(evenPipeData[3]), .CK(clk), .Q(ev0Data[3]) );
  DFF_X1 \ev0Data_reg[2]  ( .D(evenPipeData[2]), .CK(clk), .Q(ev0Data[2]) );
  DFF_X1 \ev0Data_reg[1]  ( .D(evenPipeData[1]), .CK(clk), .Q(ev0Data[1]) );
  DFF_X1 \ev0Data_reg[0]  ( .D(evenPipeData[0]), .CK(clk), .Q(ev0Data[0]) );
  DFF_X1 \od0Data_reg[31]  ( .D(oddPipeData[31]), .CK(clk), .Q(od0Data[31]) );
  DFF_X1 \od0Data_reg[30]  ( .D(oddPipeData[30]), .CK(clk), .Q(od0Data[30]) );
  DFF_X1 \od0Data_reg[29]  ( .D(oddPipeData[29]), .CK(clk), .Q(od0Data[29]) );
  DFF_X1 \od0Data_reg[28]  ( .D(oddPipeData[28]), .CK(clk), .Q(od0Data[28]) );
  DFF_X1 \od0Data_reg[27]  ( .D(oddPipeData[27]), .CK(clk), .Q(od0Data[27]) );
  DFF_X1 \od0Data_reg[26]  ( .D(oddPipeData[26]), .CK(clk), .Q(od0Data[26]) );
  DFF_X1 \od0Data_reg[25]  ( .D(oddPipeData[25]), .CK(clk), .Q(od0Data[25]) );
  DFF_X1 \od0Data_reg[24]  ( .D(oddPipeData[24]), .CK(clk), .Q(od0Data[24]) );
  DFF_X1 \od0Data_reg[23]  ( .D(oddPipeData[23]), .CK(clk), .Q(od0Data[23]) );
  DFF_X1 \od0Data_reg[22]  ( .D(oddPipeData[22]), .CK(clk), .Q(od0Data[22]) );
  DFF_X1 \od0Data_reg[21]  ( .D(oddPipeData[21]), .CK(clk), .Q(od0Data[21]) );
  DFF_X1 \od0Data_reg[20]  ( .D(oddPipeData[20]), .CK(clk), .Q(od0Data[20]) );
  DFF_X1 \od0Data_reg[19]  ( .D(oddPipeData[19]), .CK(clk), .Q(od0Data[19]) );
  DFF_X1 \od0Data_reg[18]  ( .D(oddPipeData[18]), .CK(clk), .Q(od0Data[18]) );
  DFF_X1 \od0Data_reg[17]  ( .D(oddPipeData[17]), .CK(clk), .Q(od0Data[17]) );
  DFF_X1 \od0Data_reg[16]  ( .D(oddPipeData[16]), .CK(clk), .Q(od0Data[16]) );
  DFF_X1 \od0Data_reg[15]  ( .D(oddPipeData[15]), .CK(clk), .Q(od0Data[15]) );
  DFF_X1 \od0Data_reg[14]  ( .D(oddPipeData[14]), .CK(clk), .Q(od0Data[14]) );
  DFF_X1 \od0Data_reg[13]  ( .D(oddPipeData[13]), .CK(clk), .Q(od0Data[13]) );
  DFF_X1 \od0Data_reg[12]  ( .D(oddPipeData[12]), .CK(clk), .Q(od0Data[12]) );
  DFF_X1 \od0Data_reg[11]  ( .D(oddPipeData[11]), .CK(clk), .Q(od0Data[11]) );
  DFF_X1 \od0Data_reg[10]  ( .D(oddPipeData[10]), .CK(clk), .Q(od0Data[10]) );
  DFF_X1 \od0Data_reg[9]  ( .D(oddPipeData[9]), .CK(clk), .Q(od0Data[9]) );
  DFF_X1 \od0Data_reg[8]  ( .D(oddPipeData[8]), .CK(clk), .Q(od0Data[8]) );
  DFF_X1 \od0Data_reg[7]  ( .D(oddPipeData[7]), .CK(clk), .Q(od0Data[7]) );
  DFF_X1 \od0Data_reg[6]  ( .D(oddPipeData[6]), .CK(clk), .Q(od0Data[6]) );
  DFF_X1 \od0Data_reg[5]  ( .D(oddPipeData[5]), .CK(clk), .Q(od0Data[5]) );
  DFF_X1 \od0Data_reg[4]  ( .D(oddPipeData[4]), .CK(clk), .Q(od0Data[4]) );
  DFF_X1 \od0Data_reg[3]  ( .D(oddPipeData[3]), .CK(clk), .Q(od0Data[3]) );
  DFF_X1 \od0Data_reg[2]  ( .D(oddPipeData[2]), .CK(clk), .Q(od0Data[2]) );
  DFF_X1 \od0Data_reg[1]  ( .D(oddPipeData[1]), .CK(clk), .Q(od0Data[1]) );
  DFF_X1 \od0Data_reg[0]  ( .D(oddPipeData[0]), .CK(clk), .Q(od0Data[0]) );
  DFF_X1 \od1Data_reg[31]  ( .D(od0Data[31]), .CK(clk), .Q(od1Data[31]) );
  DFF_X1 \od1Data_reg[30]  ( .D(od0Data[30]), .CK(clk), .Q(od1Data[30]) );
  DFF_X1 \od1Data_reg[29]  ( .D(od0Data[29]), .CK(clk), .Q(od1Data[29]) );
  DFF_X1 \od1Data_reg[28]  ( .D(od0Data[28]), .CK(clk), .Q(od1Data[28]) );
  DFF_X1 \od1Data_reg[27]  ( .D(od0Data[27]), .CK(clk), .Q(od1Data[27]) );
  DFF_X1 \od1Data_reg[26]  ( .D(od0Data[26]), .CK(clk), .Q(od1Data[26]) );
  DFF_X1 \od1Data_reg[25]  ( .D(od0Data[25]), .CK(clk), .Q(od1Data[25]) );
  DFF_X1 \od1Data_reg[24]  ( .D(od0Data[24]), .CK(clk), .Q(od1Data[24]) );
  DFF_X1 \od1Data_reg[23]  ( .D(od0Data[23]), .CK(clk), .Q(od1Data[23]) );
  DFF_X1 \od1Data_reg[22]  ( .D(od0Data[22]), .CK(clk), .Q(od1Data[22]) );
  DFF_X1 \od1Data_reg[21]  ( .D(od0Data[21]), .CK(clk), .Q(od1Data[21]) );
  DFF_X1 \od1Data_reg[20]  ( .D(od0Data[20]), .CK(clk), .Q(od1Data[20]) );
  DFF_X1 \od1Data_reg[19]  ( .D(od0Data[19]), .CK(clk), .Q(od1Data[19]) );
  DFF_X1 \od1Data_reg[18]  ( .D(od0Data[18]), .CK(clk), .Q(od1Data[18]) );
  DFF_X1 \od1Data_reg[17]  ( .D(od0Data[17]), .CK(clk), .Q(od1Data[17]) );
  DFF_X1 \od1Data_reg[16]  ( .D(od0Data[16]), .CK(clk), .Q(od1Data[16]) );
  DFF_X1 \od1Data_reg[15]  ( .D(od0Data[15]), .CK(clk), .Q(od1Data[15]) );
  DFF_X1 \od1Data_reg[14]  ( .D(od0Data[14]), .CK(clk), .Q(od1Data[14]) );
  DFF_X1 \od1Data_reg[13]  ( .D(od0Data[13]), .CK(clk), .Q(od1Data[13]) );
  DFF_X1 \od1Data_reg[12]  ( .D(od0Data[12]), .CK(clk), .Q(od1Data[12]) );
  DFF_X1 \od1Data_reg[11]  ( .D(od0Data[11]), .CK(clk), .Q(od1Data[11]) );
  DFF_X1 \od1Data_reg[10]  ( .D(od0Data[10]), .CK(clk), .Q(od1Data[10]) );
  DFF_X1 \od1Data_reg[9]  ( .D(od0Data[9]), .CK(clk), .Q(od1Data[9]) );
  DFF_X1 \od1Data_reg[8]  ( .D(od0Data[8]), .CK(clk), .Q(od1Data[8]) );
  DFF_X1 \od1Data_reg[7]  ( .D(od0Data[7]), .CK(clk), .Q(od1Data[7]) );
  DFF_X1 \od1Data_reg[6]  ( .D(od0Data[6]), .CK(clk), .Q(od1Data[6]) );
  DFF_X1 \od1Data_reg[5]  ( .D(od0Data[5]), .CK(clk), .Q(od1Data[5]) );
  DFF_X1 \od1Data_reg[4]  ( .D(od0Data[4]), .CK(clk), .Q(od1Data[4]) );
  DFF_X1 \od1Data_reg[3]  ( .D(od0Data[3]), .CK(clk), .Q(od1Data[3]) );
  DFF_X1 \od1Data_reg[2]  ( .D(od0Data[2]), .CK(clk), .Q(od1Data[2]) );
  DFF_X1 \od1Data_reg[1]  ( .D(od0Data[1]), .CK(clk), .Q(od1Data[1]) );
  DFF_X1 \od1Data_reg[0]  ( .D(od0Data[0]), .CK(clk), .Q(od1Data[0]) );
  CLKBUF_X1 U3 ( .A(octrl[0]), .Z(n3) );
  CLKBUF_X1 U4 ( .A(octrl[1]), .Z(n2) );
  INV_X1 U5 ( .A(rst), .ZN(n1) );
  AND2_X1 U6 ( .A1(ctrlPipe[0]), .A2(n1), .ZN(N11) );
  AND2_X1 U7 ( .A1(ctrlPipe[1]), .A2(n1), .ZN(N12) );
  AND2_X1 U8 ( .A1(actPipe), .A2(n1), .ZN(N10) );
  AND2_X1 U9 ( .A1(ictrl[0]), .A2(n1), .ZN(N8) );
  AND2_X1 U10 ( .A1(ictrl[1]), .A2(n1), .ZN(N9) );
  AND2_X1 U11 ( .A1(iact), .A2(n1), .ZN(N7) );
  MUX2_X1 U12 ( .A(evenPipeData[0]), .B(od0Data[0]), .S(n2), .Z(oOddData[0])
         );
  MUX2_X1 U13 ( .A(evenPipeData[1]), .B(od0Data[1]), .S(n2), .Z(oOddData[1])
         );
  MUX2_X1 U14 ( .A(evenPipeData[2]), .B(od0Data[2]), .S(n2), .Z(oOddData[2])
         );
  MUX2_X1 U15 ( .A(evenPipeData[3]), .B(od0Data[3]), .S(n2), .Z(oOddData[3])
         );
  MUX2_X1 U16 ( .A(evenPipeData[4]), .B(od0Data[4]), .S(n2), .Z(oOddData[4])
         );
  MUX2_X1 U17 ( .A(evenPipeData[5]), .B(od0Data[5]), .S(n2), .Z(oOddData[5])
         );
  MUX2_X1 U18 ( .A(evenPipeData[6]), .B(od0Data[6]), .S(n2), .Z(oOddData[6])
         );
  MUX2_X1 U19 ( .A(evenPipeData[7]), .B(od0Data[7]), .S(n2), .Z(oOddData[7])
         );
  MUX2_X1 U20 ( .A(evenPipeData[8]), .B(od0Data[8]), .S(octrl[1]), .Z(
        oOddData[8]) );
  MUX2_X1 U21 ( .A(evenPipeData[9]), .B(od0Data[9]), .S(octrl[1]), .Z(
        oOddData[9]) );
  MUX2_X1 U22 ( .A(evenPipeData[10]), .B(od0Data[10]), .S(octrl[1]), .Z(
        oOddData[10]) );
  MUX2_X1 U23 ( .A(evenPipeData[11]), .B(od0Data[11]), .S(octrl[1]), .Z(
        oOddData[11]) );
  MUX2_X1 U24 ( .A(evenPipeData[12]), .B(od0Data[12]), .S(octrl[1]), .Z(
        oOddData[12]) );
  MUX2_X1 U25 ( .A(evenPipeData[13]), .B(od0Data[13]), .S(n2), .Z(oOddData[13]) );
  MUX2_X1 U26 ( .A(evenPipeData[14]), .B(od0Data[14]), .S(n2), .Z(oOddData[14]) );
  MUX2_X1 U27 ( .A(evenPipeData[15]), .B(od0Data[15]), .S(n2), .Z(oOddData[15]) );
  MUX2_X1 U28 ( .A(evenPipeData[16]), .B(od0Data[16]), .S(n2), .Z(oOddData[16]) );
  MUX2_X1 U29 ( .A(evenPipeData[17]), .B(od0Data[17]), .S(octrl[1]), .Z(
        oOddData[17]) );
  MUX2_X1 U30 ( .A(evenPipeData[18]), .B(od0Data[18]), .S(octrl[1]), .Z(
        oOddData[18]) );
  MUX2_X1 U31 ( .A(evenPipeData[19]), .B(od0Data[19]), .S(octrl[1]), .Z(
        oOddData[19]) );
  MUX2_X1 U32 ( .A(evenPipeData[20]), .B(od0Data[20]), .S(octrl[1]), .Z(
        oOddData[20]) );
  MUX2_X1 U33 ( .A(evenPipeData[21]), .B(od0Data[21]), .S(octrl[1]), .Z(
        oOddData[21]) );
  MUX2_X1 U34 ( .A(evenPipeData[22]), .B(od0Data[22]), .S(n2), .Z(oOddData[22]) );
  MUX2_X1 U35 ( .A(evenPipeData[23]), .B(od0Data[23]), .S(octrl[1]), .Z(
        oOddData[23]) );
  MUX2_X1 U36 ( .A(evenPipeData[24]), .B(od0Data[24]), .S(octrl[1]), .Z(
        oOddData[24]) );
  MUX2_X1 U37 ( .A(evenPipeData[25]), .B(od0Data[25]), .S(octrl[1]), .Z(
        oOddData[25]) );
  MUX2_X1 U38 ( .A(evenPipeData[26]), .B(od0Data[26]), .S(octrl[1]), .Z(
        oOddData[26]) );
  MUX2_X1 U39 ( .A(evenPipeData[27]), .B(od0Data[27]), .S(octrl[1]), .Z(
        oOddData[27]) );
  MUX2_X1 U40 ( .A(evenPipeData[28]), .B(od0Data[28]), .S(octrl[1]), .Z(
        oOddData[28]) );
  MUX2_X1 U41 ( .A(evenPipeData[29]), .B(od0Data[29]), .S(octrl[1]), .Z(
        oOddData[29]) );
  MUX2_X1 U42 ( .A(evenPipeData[30]), .B(od0Data[30]), .S(octrl[1]), .Z(
        oOddData[30]) );
  MUX2_X1 U43 ( .A(evenPipeData[31]), .B(od0Data[31]), .S(octrl[1]), .Z(
        oOddData[31]) );
  MUX2_X1 U44 ( .A(ev0Data[0]), .B(od1Data[0]), .S(n3), .Z(oEvenData[0]) );
  MUX2_X1 U45 ( .A(ev0Data[1]), .B(od1Data[1]), .S(n3), .Z(oEvenData[1]) );
  MUX2_X1 U46 ( .A(ev0Data[2]), .B(od1Data[2]), .S(n3), .Z(oEvenData[2]) );
  MUX2_X1 U47 ( .A(ev0Data[3]), .B(od1Data[3]), .S(n3), .Z(oEvenData[3]) );
  MUX2_X1 U48 ( .A(ev0Data[4]), .B(od1Data[4]), .S(n3), .Z(oEvenData[4]) );
  MUX2_X1 U49 ( .A(ev0Data[5]), .B(od1Data[5]), .S(n3), .Z(oEvenData[5]) );
  MUX2_X1 U50 ( .A(ev0Data[6]), .B(od1Data[6]), .S(n3), .Z(oEvenData[6]) );
  MUX2_X1 U51 ( .A(ev0Data[7]), .B(od1Data[7]), .S(n3), .Z(oEvenData[7]) );
  MUX2_X1 U52 ( .A(ev0Data[8]), .B(od1Data[8]), .S(octrl[0]), .Z(oEvenData[8])
         );
  MUX2_X1 U53 ( .A(ev0Data[9]), .B(od1Data[9]), .S(octrl[0]), .Z(oEvenData[9])
         );
  MUX2_X1 U54 ( .A(ev0Data[10]), .B(od1Data[10]), .S(octrl[0]), .Z(
        oEvenData[10]) );
  MUX2_X1 U55 ( .A(ev0Data[11]), .B(od1Data[11]), .S(octrl[0]), .Z(
        oEvenData[11]) );
  MUX2_X1 U56 ( .A(ev0Data[12]), .B(od1Data[12]), .S(octrl[0]), .Z(
        oEvenData[12]) );
  MUX2_X1 U57 ( .A(ev0Data[13]), .B(od1Data[13]), .S(n3), .Z(oEvenData[13]) );
  MUX2_X1 U58 ( .A(ev0Data[14]), .B(od1Data[14]), .S(n3), .Z(oEvenData[14]) );
  MUX2_X1 U59 ( .A(ev0Data[15]), .B(od1Data[15]), .S(n3), .Z(oEvenData[15]) );
  MUX2_X1 U60 ( .A(ev0Data[16]), .B(od1Data[16]), .S(n3), .Z(oEvenData[16]) );
  MUX2_X1 U61 ( .A(ev0Data[17]), .B(od1Data[17]), .S(octrl[0]), .Z(
        oEvenData[17]) );
  MUX2_X1 U62 ( .A(ev0Data[18]), .B(od1Data[18]), .S(octrl[0]), .Z(
        oEvenData[18]) );
  MUX2_X1 U63 ( .A(ev0Data[19]), .B(od1Data[19]), .S(octrl[0]), .Z(
        oEvenData[19]) );
  MUX2_X1 U64 ( .A(ev0Data[20]), .B(od1Data[20]), .S(octrl[0]), .Z(
        oEvenData[20]) );
  MUX2_X1 U65 ( .A(ev0Data[21]), .B(od1Data[21]), .S(octrl[0]), .Z(
        oEvenData[21]) );
  MUX2_X1 U66 ( .A(ev0Data[22]), .B(od1Data[22]), .S(n3), .Z(oEvenData[22]) );
  MUX2_X1 U67 ( .A(ev0Data[23]), .B(od1Data[23]), .S(octrl[0]), .Z(
        oEvenData[23]) );
  MUX2_X1 U68 ( .A(ev0Data[24]), .B(od1Data[24]), .S(octrl[0]), .Z(
        oEvenData[24]) );
  MUX2_X1 U69 ( .A(ev0Data[25]), .B(od1Data[25]), .S(octrl[0]), .Z(
        oEvenData[25]) );
  MUX2_X1 U70 ( .A(ev0Data[26]), .B(od1Data[26]), .S(octrl[0]), .Z(
        oEvenData[26]) );
  MUX2_X1 U71 ( .A(ev0Data[27]), .B(od1Data[27]), .S(octrl[0]), .Z(
        oEvenData[27]) );
  MUX2_X1 U72 ( .A(ev0Data[28]), .B(od1Data[28]), .S(octrl[0]), .Z(
        oEvenData[28]) );
  MUX2_X1 U73 ( .A(ev0Data[29]), .B(od1Data[29]), .S(octrl[0]), .Z(
        oEvenData[29]) );
  MUX2_X1 U74 ( .A(ev0Data[30]), .B(od1Data[30]), .S(octrl[0]), .Z(
        oEvenData[30]) );
  MUX2_X1 U75 ( .A(ev0Data[31]), .B(od1Data[31]), .S(octrl[0]), .Z(
        oEvenData[31]) );
endmodule


module radix2Butterfly_FFT_DW16_FFT_N10_PL_DEPTH3 ( clk, rst, iact, ictrl, 
        oact, octrl, iMemAddr, oMemAddr, opa_real, opa_imag, opb_real, 
        opb_imag, twiddle_real, twiddle_imag, dst_opa_real, dst_opa_imag, 
        dst_opb_real, dst_opb_imag );
  input [1:0] ictrl;
  output [1:0] octrl;
  input [8:0] iMemAddr;
  output [8:0] oMemAddr;
  input [15:0] opa_real;
  input [15:0] opa_imag;
  input [15:0] opb_real;
  input [15:0] opb_imag;
  input [16:0] twiddle_real;
  input [16:0] twiddle_imag;
  output [15:0] dst_opa_real;
  output [15:0] dst_opa_imag;
  output [15:0] dst_opb_real;
  output [15:0] dst_opb_imag;
  input clk, rst, iact;
  output oact;
  wire   \yr[31] , \yi[31] , N78, N79, N80, N81, N82, N83, N84, N85, N86, N87,
         N88, N89, N90, N91, N92, N93, N94, N95, N96, N97, N98, N99, N100,
         N101, N102, N103, N104, N105, N106, N107, N108, N109, N111, N112,
         \intadd_1/B[14] , \intadd_1/B[13] , \intadd_1/B[12] ,
         \intadd_1/B[11] , \intadd_1/B[10] , \intadd_1/B[9] , \intadd_1/B[8] ,
         \intadd_1/B[7] , \intadd_1/B[6] , \intadd_1/B[5] , \intadd_1/B[4] ,
         \intadd_1/B[3] , \intadd_1/B[2] , \intadd_1/B[1] , \intadd_1/B[0] ,
         \intadd_1/CI , \intadd_1/SUM[14] , \intadd_1/SUM[13] ,
         \intadd_1/SUM[12] , \intadd_1/SUM[11] , \intadd_1/SUM[10] ,
         \intadd_1/SUM[9] , \intadd_1/SUM[8] , \intadd_1/SUM[7] ,
         \intadd_1/SUM[6] , \intadd_1/SUM[5] , \intadd_1/SUM[4] ,
         \intadd_1/SUM[3] , \intadd_1/SUM[2] , \intadd_1/SUM[1] ,
         \intadd_1/SUM[0] , \intadd_1/n15 , \intadd_1/n14 , \intadd_1/n13 ,
         \intadd_1/n12 , \intadd_1/n11 , \intadd_1/n10 , \intadd_1/n9 ,
         \intadd_1/n8 , \intadd_1/n7 , \intadd_1/n6 , \intadd_1/n5 ,
         \intadd_1/n4 , \intadd_1/n3 , \intadd_1/n2 , \intadd_1/n1 ,
         \intadd_2/CI , \intadd_2/SUM[14] , \intadd_2/SUM[13] ,
         \intadd_2/SUM[12] , \intadd_2/SUM[11] , \intadd_2/SUM[10] ,
         \intadd_2/SUM[9] , \intadd_2/SUM[8] , \intadd_2/SUM[7] ,
         \intadd_2/SUM[6] , \intadd_2/SUM[5] , \intadd_2/SUM[4] ,
         \intadd_2/SUM[3] , \intadd_2/SUM[2] , \intadd_2/SUM[1] ,
         \intadd_2/SUM[0] , \intadd_2/n15 , \intadd_2/n14 , \intadd_2/n13 ,
         \intadd_2/n12 , \intadd_2/n11 , \intadd_2/n10 , \intadd_2/n9 ,
         \intadd_2/n8 , \intadd_2/n7 , \intadd_2/n6 , \intadd_2/n5 ,
         \intadd_2/n4 , \intadd_2/n3 , \intadd_2/n2 , \intadd_2/n1 ,
         \intadd_3/A[14] , \intadd_3/B[14] , \intadd_3/CI , \intadd_3/n15 ,
         \intadd_3/n14 , \intadd_3/n13 , \intadd_3/n12 , \intadd_3/n11 ,
         \intadd_3/n10 , \intadd_3/n9 , \intadd_3/n8 , \intadd_3/n7 ,
         \intadd_3/n6 , \intadd_3/n5 , \intadd_3/n4 , \intadd_3/n3 ,
         \intadd_3/n2 , \intadd_3/n1 , \intadd_4/CI , \intadd_4/SUM[13] ,
         \intadd_4/SUM[12] , \intadd_4/SUM[11] , \intadd_4/SUM[10] ,
         \intadd_4/SUM[9] , \intadd_4/SUM[8] , \intadd_4/SUM[7] ,
         \intadd_4/SUM[6] , \intadd_4/SUM[5] , \intadd_4/SUM[4] ,
         \intadd_4/SUM[3] , \intadd_4/SUM[2] , \intadd_4/SUM[1] ,
         \intadd_4/SUM[0] , \intadd_4/n14 , \intadd_4/n13 , \intadd_4/n12 ,
         \intadd_4/n11 , \intadd_4/n10 , \intadd_4/n9 , \intadd_4/n8 ,
         \intadd_4/n7 , \intadd_4/n6 , \intadd_4/n5 , \intadd_4/n4 ,
         \intadd_4/n3 , \intadd_4/n2 , \intadd_4/n1 , \intadd_5/A[10] ,
         \intadd_5/A[9] , \intadd_5/A[8] , \intadd_5/A[7] , \intadd_5/A[6] ,
         \intadd_5/A[5] , \intadd_5/A[4] , \intadd_5/A[3] , \intadd_5/A[2] ,
         \intadd_5/A[1] , \intadd_5/A[0] , \intadd_5/B[10] , \intadd_5/B[9] ,
         \intadd_5/B[8] , \intadd_5/B[7] , \intadd_5/B[6] , \intadd_5/B[5] ,
         \intadd_5/B[4] , \intadd_5/B[3] , \intadd_5/B[2] , \intadd_5/B[1] ,
         \intadd_5/B[0] , \intadd_5/CI , \intadd_5/SUM[10] , \intadd_5/SUM[9] ,
         \intadd_5/SUM[8] , \intadd_5/SUM[7] , \intadd_5/SUM[6] ,
         \intadd_5/SUM[5] , \intadd_5/SUM[4] , \intadd_5/SUM[3] ,
         \intadd_5/SUM[2] , \intadd_5/SUM[1] , \intadd_5/SUM[0] ,
         \intadd_5/n11 , \intadd_5/n10 , \intadd_5/n9 , \intadd_5/n8 ,
         \intadd_5/n7 , \intadd_5/n6 , \intadd_5/n5 , \intadd_5/n4 ,
         \intadd_5/n3 , \intadd_5/n2 , \intadd_5/n1 , \intadd_6/A[8] ,
         \intadd_6/A[7] , \intadd_6/A[6] , \intadd_6/A[5] , \intadd_6/A[4] ,
         \intadd_6/A[3] , \intadd_6/A[2] , \intadd_6/A[1] , \intadd_6/A[0] ,
         \intadd_6/B[8] , \intadd_6/B[7] , \intadd_6/B[5] , \intadd_6/B[4] ,
         \intadd_6/B[3] , \intadd_6/B[2] , \intadd_6/B[1] , \intadd_6/B[0] ,
         \intadd_6/CI , \intadd_6/SUM[8] , \intadd_6/SUM[7] ,
         \intadd_6/SUM[6] , \intadd_6/SUM[5] , \intadd_6/SUM[4] ,
         \intadd_6/SUM[3] , \intadd_6/SUM[2] , \intadd_6/SUM[1] ,
         \intadd_6/SUM[0] , \intadd_6/n9 , \intadd_6/n8 , \intadd_6/n7 ,
         \intadd_6/n6 , \intadd_6/n5 , \intadd_6/n4 , \intadd_6/n3 ,
         \intadd_6/n2 , \intadd_6/n1 , \intadd_7/A[2] , \intadd_7/A[1] ,
         \intadd_7/A[0] , \intadd_7/B[8] , \intadd_7/B[7] , \intadd_7/B[6] ,
         \intadd_7/B[5] , \intadd_7/B[4] , \intadd_7/B[3] , \intadd_7/B[2] ,
         \intadd_7/B[1] , \intadd_7/B[0] , \intadd_7/CI , \intadd_7/SUM[8] ,
         \intadd_7/n9 , \intadd_7/n8 , \intadd_7/n7 , \intadd_7/n6 ,
         \intadd_7/n5 , \intadd_7/n4 , \intadd_7/n3 , \intadd_7/n2 ,
         \intadd_7/n1 , \intadd_8/A[6] , \intadd_8/A[5] , \intadd_8/A[4] ,
         \intadd_8/A[3] , \intadd_8/A[2] , \intadd_8/A[1] , \intadd_8/A[0] ,
         \intadd_8/B[6] , \intadd_8/B[5] , \intadd_8/B[4] , \intadd_8/B[3] ,
         \intadd_8/B[2] , \intadd_8/B[1] , \intadd_8/B[0] , \intadd_8/CI ,
         \intadd_8/SUM[6] , \intadd_8/SUM[5] , \intadd_8/SUM[4] ,
         \intadd_8/SUM[3] , \intadd_8/SUM[2] , \intadd_8/SUM[1] ,
         \intadd_8/SUM[0] , \intadd_8/n7 , \intadd_8/n6 , \intadd_8/n5 ,
         \intadd_8/n4 , \intadd_8/n3 , \intadd_8/n2 , \intadd_8/n1 ,
         \intadd_9/A[3] , \intadd_9/A[2] , \intadd_9/A[1] , \intadd_9/A[0] ,
         \intadd_9/B[3] , \intadd_9/B[2] , \intadd_9/B[1] , \intadd_9/B[0] ,
         \intadd_9/CI , \intadd_9/SUM[3] , \intadd_9/SUM[2] ,
         \intadd_9/SUM[1] , \intadd_9/SUM[0] , \intadd_9/n4 , \intadd_9/n3 ,
         \intadd_9/n2 , \intadd_9/n1 , \intadd_10/A[1] , \intadd_10/A[0] ,
         \intadd_10/B[2] , \intadd_10/B[1] , \intadd_10/B[0] , \intadd_10/CI ,
         \intadd_10/n3 , \intadd_10/n2 , \intadd_10/n1 , \intadd_11/A[0] ,
         \intadd_11/B[2] , \intadd_11/B[1] , \intadd_11/B[0] , \intadd_11/CI ,
         \intadd_11/SUM[2] , \intadd_11/n3 , \intadd_11/n2 , \intadd_11/n1 ,
         \intadd_12/B[2] , \intadd_12/B[1] , \intadd_12/B[0] , \intadd_12/CI ,
         \intadd_12/SUM[2] , \intadd_12/n3 , \intadd_12/n2 , \intadd_12/n1 ,
         \intadd_0/A[26] , \intadd_0/A[25] , \intadd_0/A[24] ,
         \intadd_0/A[22] , \intadd_0/A[19] , \intadd_0/A[16] ,
         \intadd_0/A[15] , \intadd_0/A[2] , \intadd_0/A[1] , \intadd_0/A[0] ,
         \intadd_0/B[26] , \intadd_0/B[25] , \intadd_0/B[23] ,
         \intadd_0/B[20] , \intadd_0/B[17] , \intadd_0/B[16] ,
         \intadd_0/B[13] , \intadd_0/B[12] , \intadd_0/B[11] ,
         \intadd_0/B[10] , \intadd_0/B[9] , \intadd_0/B[8] , \intadd_0/B[7] ,
         \intadd_0/B[6] , \intadd_0/B[5] , \intadd_0/B[4] , \intadd_0/B[3] ,
         \intadd_0/B[2] , \intadd_0/B[1] , \intadd_0/B[0] , \intadd_0/CI ,
         \intadd_0/SUM[26] , \intadd_0/SUM[25] , \intadd_0/SUM[24] ,
         \intadd_0/SUM[23] , \intadd_0/SUM[22] , \intadd_0/SUM[21] ,
         \intadd_0/SUM[20] , \intadd_0/SUM[19] , \intadd_0/SUM[18] ,
         \intadd_0/SUM[17] , \intadd_0/SUM[16] , \intadd_0/SUM[15] ,
         \intadd_0/SUM[14] , \intadd_0/SUM[13] , \intadd_0/SUM[12] ,
         \intadd_0/SUM[11] , \intadd_0/SUM[10] , \intadd_0/SUM[9] ,
         \intadd_0/SUM[8] , \intadd_0/SUM[7] , \intadd_0/SUM[6] ,
         \intadd_0/SUM[5] , \intadd_0/SUM[4] , \intadd_0/SUM[3] ,
         \intadd_0/SUM[2] , \intadd_0/SUM[1] , \intadd_0/SUM[0] ,
         \intadd_0/n27 , \intadd_0/n26 , \intadd_0/n25 , \intadd_0/n24 ,
         \intadd_0/n23 , \intadd_0/n22 , \intadd_0/n21 , \intadd_0/n20 ,
         \intadd_0/n19 , \intadd_0/n18 , \intadd_0/n17 , \intadd_0/n16 ,
         \intadd_0/n15 , \intadd_0/n14 , \intadd_0/n13 , \intadd_0/n12 ,
         \intadd_0/n11 , \intadd_0/n10 , \intadd_0/n9 , \intadd_0/n8 ,
         \intadd_0/n7 , \intadd_0/n6 , \intadd_0/n5 , \intadd_0/n4 ,
         \intadd_0/n3 , \intadd_0/n2 , \intadd_0/n1 , n1, n2, n3, n4, n5, n6,
         n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19, n20,
         n21, n22, n23, n24, n25, n26, n27, n28, n29, n30, n31, n32, n33, n34,
         n35, n36, n37, n38, n39, n40, n41, n42, n43, n44, n45, n46, n47, n48,
         n49, n50, n51, n52, n53, n54, n55, n56, n57, n58, n59, n60, n61, n62,
         n63, n64, n65, n66, n67, n68, n69, n70, n71, n72, n73, n74, n75, n76,
         n77, n78, n79, n80, n81, n82, n83, n84, n85, n86, n87, n88, n89, n90,
         n91, n92, n93, n94, n95, n96, n97, n98, n99, n100, n101, n102, n103,
         n104, n105, n106, n107, n108, n109, n110, n111, n112, n113, n114,
         n115, n116, n117, n118, n119, n120, n121, n122, n123, n124, n125,
         n126, n127, n128, n129, n130, n131, n132, n133, n134, n135, n136,
         n137, n138, n139, n140, n141, n142, n143, n144, n145, n146, n147,
         n148, n149, n150, n151, n152, n153, n154, n155, n156, n157, n158,
         n159, n160, n161, n162, n163, n164, n165, n166, n167, n168, n169,
         n170, n171, n172, n173, n174, n175, n176, n177, n178, n179, n180,
         n181, n182, n183, n184, n185, n186, n187, n188, n189, n190, n191,
         n192, n193, n194, n195, n196, n197, n198, n199, n200, n201, n202,
         n203, n204, n205, n206, n207, n208, n209, n210, n211, n212, n213,
         n214, n215, n216, n217, n218, n219, n220, n221, n222, n223, n224,
         n225, n226, n227, n228, n229, n230, n231, n232, n233, n234, n235,
         n236, n237, n238, n239, n240, n241, n242, n243, n244, n245, n246,
         n247, n248, n249, n250, n251, n252, n253, n254, n255, n256, n257,
         n258, n259, n260, n261, n262, n263, n264, n265, n266, n267, n268,
         n269, n270, n271, n272, n273, n274, n275, n276, n277, n278, n279,
         n280, n281, n282, n283, n284, n285, n286, n287, n288, n289, n290,
         n291, n292, n293, n294, n295, n296, n297, n298, n299, n300, n301,
         n302, n303, n304, n305, n306, n307, n308, n309, n310, n311, n312,
         n313, n314, n315, n316, n317, n318, n319, n320, n321, n322, n323,
         n324, n325, n326, n327, n328, n329, n330, n331, n332, n333, n334,
         n335, n336, n337, n338, n339, n340, n341, n342, n343, n344, n345,
         n346, n347, n348, n349, n350, n351, n352, n353, n354, n355, n356,
         n357, n358, n359, n360, n361, n362, n363, n364, n365, n366, n367,
         n368, n369, n370, n371, n372, n373, n374, n375, n376, n377, n378,
         n379, n380, n381, n382, n383, n384, n385, n386, n387, n388, n389,
         n390, n391, n392, n393, n394, n395, n396, n397, n398, n399, n400,
         n401, n402, n403, n404, n405, n406, n407, n408, n409, n410, n411,
         n412, n413, n414, n415, n416, n417, n418, n419, n420, n421, n422,
         n423, n424, n425, n426, n427, n428, n429, n430, n431, n432, n433,
         n434, n435, n436, n437, n438, n439, n440, n441, n442, n443, n444,
         n445, n446, n447, n448, n449, n450, n451, n452, n453, n454, n455,
         n456, n457, n458, n459, n460, n461, n462, n463, n464, n465, n466,
         n467, n468, n469, n470, n471, n472, n473, n474, n475, n476, n477,
         n478, n479, n480, n481, n482, n483, n484, n485, n486, n487, n488,
         n489, n490, n491, n492, n493, n494, n495, n496, n497, n498, n499,
         n500, n501, n502, n503, n504, n505, n506, n507, n508, n509, n510,
         n511, n512, n513, n514, n515, n516, n517, n518, n519, n520, n521,
         n522, n523, n524, n525, n526, n527, n528, n529, n530, n531, n532,
         n533, n534, n535, n536, n537, n538, n539, n540, n541, n542, n543,
         n544, n545, n546, n547, n548, n549, n550, n551, n552, n553, n554,
         n555, n556, n557, n558, n559, n560, n561, n562, n563, n564, n565,
         n566, n567, n568, n569, n570, n571, n572, n573, n574, n575, n576,
         n577, n578, n579, n580, n581, n582, n583, n584, n585, n586, n587,
         n588, n589, n590, n591, n592, n593, n594, n595, n596, n597, n598,
         n599, n600, n601, n602, n603, n604, n605, n606, n607, n608, n609,
         n610, n611, n612, n613, n614, n615, n616, n617, n618, n619, n620,
         n621, n622, n623, n624, n625, n626, n627, n628, n629, n630, n631,
         n632, n633, n634, n635, n636, n637, n638, n639, n640, n641, n642,
         n643, n644, n645, n646, n647, n648, n649, n650, n651, n652, n653,
         n654, n655, n656, n657, n658, n659, n660, n661, n662, n663, n664,
         n665, n666, n667, n668, n669, n670, n671, n672, n673, n674, n675,
         n676, n677, n678, n679, n680, n681, n682, n683, n684, n685, n686,
         n687, n688, n689, n690, n691, n692, n693, n694, n695, n696, n697,
         n698, n699, n700, n701, n702, n703, n704, n705, n706, n707, n708,
         n709, n710, n711, n712, n713, n714, n715, n716, n717, n718, n719,
         n720, n721, n722, n723, n724, n725, n726, n727, n728, n729, n730,
         n731, n732, n733, n734, n735, n736, n737, n738, n739, n740, n741,
         n742, n743, n744, n745, n746, n747, n748, n749, n750, n751, n752,
         n753, n754, n755, n756, n757, n758, n759, n760, n761, n762, n763,
         n764, n765, n766, n767, n768, n769, n770, n771, n772, n773, n774,
         n775, n776, n777, n778, n779, n780, n781, n782, n783, n784, n785,
         n786, n787, n788, n789, n790, n791, n792, n793, n794, n795, n796,
         n797, n798, n799, n800, n801, n802, n803, n804, n805, n806, n807,
         n808, n809, n810, n811, n812, n813, n814, n815, n816, n817, n818,
         n819, n820, n821, n822, n823, n824, n825, n826, n827, n828, n829,
         n830, n831, n832, n833, n834, n835, n836, n837, n838, n839, n840,
         n841, n842, n843, n844, n845, n846, n847, n848, n849, n850, n851,
         n852, n853, n854, n855, n856, n857, n858, n859, n860, n861, n862,
         n863, n864, n865, n866, n867, n868, n869, n870, n871, n872, n873,
         n874, n875, n876, n877, n878, n879, n880, n881, n882, n883, n884,
         n885, n886, n887, n888, n889, n890, n891, n892, n893, n894, n895,
         n896, n897, n898, n899, n900, n901, n902, n903, n904, n905, n906,
         n907, n908, n909, n910, n911, n912, n913, n914, n915, n916, n917,
         n918, n919, n920, n921, n922, n923, n924, n925, n926, n927, n928,
         n929, n930, n931, n932, n933, n934, n935, n936, n937, n938, n939,
         n940, n941, n942, n943, n944, n945, n946, n947, n948, n949, n950,
         n951, n952, n953, n954, n955, n956, n957, n958, n959, n960, n961,
         n962, n963, n964, n965, n966, n967, n968, n969, n970, n971, n972,
         n973, n974, n975, n976, n977, n978, n979, n980, n981, n982, n983,
         n984, n985, n986, n987, n988, n989, n990, n991, n992, n993, n994,
         n995, n996, n997, n998, n999, n1000, n1001, n1002, n1003, n1004,
         n1005, n1006, n1007, n1008, n1009, n1010, n1011, n1012, n1013, n1014,
         n1015, n1016, n1017, n1018, n1019, n1020, n1021, n1022, n1023, n1024,
         n1025, n1026, n1027, n1028, n1029, n1030, n1031, n1032, n1033, n1034,
         n1035, n1036, n1037, n1038, n1039, n1040, n1041, n1042, n1043, n1044,
         n1045, n1046, n1047, n1048, n1049, n1050, n1051, n1052, n1053, n1054,
         n1055, n1056, n1057, n1058, n1059, n1060, n1061, n1062, n1063, n1064,
         n1065, n1066, n1067, n1068, n1069, n1070, n1071, n1072, n1073, n1074,
         n1075, n1076, n1077, n1078, n1079, n1080, n1081, n1082, n1083, n1084,
         n1085, n1086, n1087, n1088, n1089, n1090, n1091, n1092, n1093, n1094,
         n1095, n1096, n1097, n1098, n1099, n1100, n1101, n1102, n1103, n1104,
         n1105, n1106, n1107, n1108, n1109, n1110, n1111, n1112, n1113, n1114,
         n1115, n1116, n1117, n1118, n1119, n1120, n1121, n1122, n1123, n1124,
         n1125, n1126, n1127, n1128, n1129, n1130, n1131, n1132, n1133, n1134,
         n1135, n1136, n1137, n1138, n1139, n1140, n1141, n1142, n1143, n1144,
         n1145, n1146, n1147, n1148, n1149, n1150, n1151, n1152, n1153, n1154,
         n1155, n1156, n1157, n1158, n1159, n1160, n1161, n1162, n1163, n1164,
         n1165, n1166, n1167, n1168, n1169, n1170, n1171, n1172, n1173, n1174,
         n1175, n1176, n1177, n1178, n1179, n1180, n1181, n1182, n1183, n1184,
         n1185, n1186, n1187, n1188, n1189, n1190, n1191, n1192, n1193, n1194,
         n1195, n1196, n1197, n1198, n1199, n1200, n1201, n1202, n1203, n1204,
         n1205, n1206, n1207, n1208, n1209, n1210, n1211, n1212, n1213, n1214,
         n1215, n1216, n1217, n1218, n1219, n1220, n1221, n1222, n1223, n1224,
         n1225, n1226, n1227, n1228, n1229, n1230, n1231, n1232, n1233, n1234,
         n1235, n1236, n1237, n1238, n1239, n1240, n1241, n1242, n1243, n1244,
         n1245, n1246, n1247, n1248, n1249, n1250, n1251, n1252, n1253, n1254,
         n1255, n1256, n1257, n1258, n1259, n1260, n1261, n1262, n1263, n1264,
         n1265, n1266, n1267, n1268, n1269, n1270, n1271, n1272, n1273, n1274,
         n1275, n1276, n1277, n1278, n1279, n1280, n1281, n1282, n1283, n1284,
         n1285, n1286, n1287, n1288, n1289, n1290, n1291, n1292, n1293, n1294,
         n1295, n1296, n1297, n1298, n1299, n1300, n1301, n1302, n1303, n1304,
         n1305, n1306, n1307, n1308, n1309, n1310, n1311, n1312, n1313, n1314,
         n1315, n1316, n1317, n1318, n1319, n1320, n1321, n1322, n1323, n1324,
         n1325, n1326, n1327, n1328, n1329, n1330, n1331, n1332, n1333, n1334,
         n1335, n1336, n1337, n1338, n1339, n1340, n1341, n1342, n1343, n1344,
         n1345, n1346, n1347, n1348, n1349, n1350, n1351, n1352, n1353, n1354,
         n1355, n1356, n1357, n1358, n1359, n1360, n1361, n1362, n1363, n1364,
         n1365, n1366, n1367, n1368, n1369, n1370, n1371, n1372, n1373, n1374,
         n1375, n1376, n1377, n1378, n1379, n1380, n1381, n1382, n1383, n1384,
         n1385, n1386, n1387, n1388, n1389, n1390, n1391, n1392, n1393, n1394,
         n1395, n1396, n1397, n1398, n1399, n1400, n1401, n1402, n1403, n1404,
         n1405, n1406, n1407, n1408, n1409, n1410, n1411, n1412, n1413, n1414,
         n1415, n1416, n1417, n1418, n1419, n1420, n1421, n1422, n1423, n1424,
         n1425, n1426, n1427, n1428, n1429, n1430, n1431, n1432, n1433, n1434,
         n1435, n1436, n1437, n1438, n1439, n1440, n1441, n1442, n1443, n1444,
         n1445, n1446, n1447, n1448, n1449, n1450, n1451, n1452, n1453, n1454,
         n1455, n1456, n1457, n1458, n1459, n1460, n1461, n1462, n1463, n1464,
         n1465, n1466, n1467, n1468, n1469, n1470, n1471, n1472, n1473, n1474,
         n1475, n1476, n1477, n1478, n1479, n1480, n1481, n1482, n1483, n1484,
         n1485, n1486, n1487, n1488, n1489, n1490, n1491, n1492, n1493, n1494,
         n1495, n1496, n1497, n1498, n1499, n1500, n1501, n1502, n1503, n1504,
         n1505, n1506, n1507, n1508, n1509, n1510, n1511, n1512, n1513, n1514,
         n1515, n1516, n1517, n1518, n1519, n1520, n1521, n1522, n1523, n1524,
         n1525, n1526, n1527, n1528, n1529, n1530, n1531, n1532, n1533, n1534,
         n1535, n1536, n1537, n1538, n1539, n1540, n1541, n1542, n1543, n1544,
         n1545, n1546, n1547, n1548, n1549, n1550, n1551, n1552, n1553, n1554,
         n1555, n1556, n1557, n1558, n1559, n1560, n1561, n1562, n1563, n1564,
         n1565, n1566, n1567, n1568, n1569, n1570, n1571, n1572, n1573, n1574,
         n1575, n1576, n1577, n1578, n1579, n1580, n1581, n1582, n1583, n1584,
         n1585, n1586, n1587, n1588, n1589, n1590, n1591, n1592, n1593, n1594,
         n1595, n1596, n1597, n1598, n1599, n1600, n1601, n1602, n1603, n1604,
         n1605, n1606, n1607, n1608, n1609, n1610, n1611, n1612, n1613, n1614,
         n1615, n1616, n1617, n1618, n1619, n1620, n1621, n1622, n1623, n1624,
         n1625, n1626, n1627, n1628, n1629, n1630, n1631, n1632, n1633, n1634,
         n1635, n1636, n1637, n1638, n1639, n1640, n1641, n1642, n1643, n1644,
         n1645, n1646, n1647, n1648, n1649, n1650, n1651, n1652, n1653, n1654,
         n1655, n1656, n1657, n1658, n1659, n1660, n1661, n1662, n1663, n1664,
         n1665, n1666, n1667, n1668, n1669, n1670, n1671, n1672, n1673, n1674,
         n1675, n1676, n1677, n1678, n1679, n1680, n1681, n1682, n1683, n1684,
         n1685, n1686, n1687, n1688, n1689, n1690, n1691, n1692, n1693, n1694,
         n1695, n1696, n1697, n1698, n1699, n1700, n1701, n1702, n1703, n1704,
         n1705, n1706, n1707, n1708, n1709, n1710, n1711, n1712, n1713, n1714,
         n1715, n1716, n1717, n1718, n1719, n1720, n1721, n1722, n1723, n1724,
         n1725, n1726, n1727, n1728, n1729, n1730, n1731, n1732, n1733, n1734,
         n1735, n1736, n1737, n1738, n1739, n1740, n1741, n1742, n1743, n1744,
         n1745, n1746, n1747, n1748, n1749, n1750, n1751, n1752, n1753, n1754,
         n1755, n1756, n1757, n1758, n1759, n1760, n1761, n1762, n1763, n1764,
         n1765, n1766, n1767, n1768, n1769, n1770, n1771, n1772, n1773, n1774,
         n1775, n1776, n1777, n1778, n1779, n1780, n1781, n1782, n1783, n1784,
         n1785, n1786, n1787, n1788, n1789, n1790, n1791, n1792, n1793, n1794,
         n1796, n1797, n1798, n1799, n1800, n1801, n1802, n1803, n1804, n1805,
         n1806, n1807, n1808, n1809, n1810, n1811, n1812, n1813, n1814, n1815,
         n1816, n1817, n1818, n1819, n1820, n1821, n1822, n1823, n1824, n1825,
         n1826, n1827, n1828, n1829, n1830, n1831, n1832, n1833, n1834, n1835,
         n1836, n1837, n1838, n1839, n1840, n1841, n1842, n1843, n1844, n1845,
         n1846, n1847, n1848, n1849, n1850, n1851, n1852, n1853, n1854, n1855,
         n1856, n1857, n1858, n1859, n1860, n1861, n1862, n1863, n1864, n1865,
         n1866, n1867, n1868, n1869, n1870, n1871, n1872, n1873, n1874, n1875,
         n1876, n1877, n1878, n1879, n1880, n1881, n1882, n1883, n1884, n1885,
         n1886, n1887, n1888, n1889, n1890, n1891, n1892, n1893, n1894, n1895,
         n1896, n1897, n1898, n1899, n1900, n1901, n1902, n1903, n1904, n1905,
         n1906, n1907, n1908, n1909, n1910, n1911, n1912, n1913, n1914, n1915,
         n1916, n1917, n1918, n1919, n1920, n1921, n1922, n1923, n1924, n1925,
         n1926, n1927, n1928, n1929, n1930, n1931, n1932, n1933, n1934, n1935,
         n1936, n1937, n1938, n1939, n1940, n1941, n1942, n1943, n1944, n1945,
         n1946, n1947, n1948, n1949, n1950, n1951, n1952, n1953, n1954, n1955,
         n1956, n1957, n1958, n1959, n1960, n1961, n1962, n1963, n1964, n1965,
         n1966, n1967, n1968, n1969, n1970, n1971, n1972, n1973, n1974, n1975,
         n1976, n1977, n1978, n1979, n1980, n1981, n1982, n1983, n1984, n1985,
         n1986, n1987, n1988, n1989, n1990, n1991, n1992, n1993, n1994, n1995,
         n1996, n1997, n1998, n1999, n2000, n2001, n2002, n2003, n2004, n2005,
         n2006, n2007, n2008, n2009, n2010, n2011, n2012, n2013, n2014, n2015,
         n2016, n2017, n2018, n2019, n2020, n2021, n2022, n2023, n2024, n2025,
         n2026, n2027, n2028, n2029, n2030, n2031, n2032, n2033, n2034, n2035,
         n2036, n2037, n2038, n2039, n2040, n2041, n2042, n2043, n2044, n2045,
         n2046, n2047, n2048, n2049, n2050, n2051, n2052, n2053, n2054, n2055,
         n2056, n2057, n2058, n2059, n2060, n2061, n2062, n2063, n2064, n2065,
         n2066, n2067, n2068, n2069, n2070;
  wire   [15:0] xbuf_real_stage1;
  wire   [15:0] xbuf_imag_stage1;
  wire   [16:0] xbuf_real_p_imag_stage1;
  wire   [17:0] twiddle_real_p_imag_stage1;
  wire   [17:0] twiddle_real_m_imag_stage1;
  wire   [15:0] xbuf_real_stage2;
  wire   [15:0] xbuf_imag_stage2;
  wire   [16:0] xbuf_real_p_imag_stage2;
  wire   [17:0] twiddle_real_p_imag_stage2;
  wire   [17:0] twiddle_real_m_imag_stage2;
  wire   [16:0] twiddle_real_stage2;
  wire   [31:0] tmp_a;
  wire   [31:0] tmp_r;
  wire   [31:0] tmp_i;
  wire   [31:0] tmp_a_stage3;
  wire   [31:0] tmp_r_stage3;
  wire   [31:0] tmp_i_stage3;
  wire   [15:0] dst_opa_real_stage2;
  wire   [15:0] dst_opa_imag_stage2;
  wire   [1:0] ictrl_stage2;
  wire   [8:0] iMemAddr_stage2;
  tri   clk;
  tri   rst;
  tri   [16:0] twiddle_real;
  tri   [16:0] twiddle_imag;
  assign dst_opb_real[15] = \yr[31] ;
  assign dst_opb_imag[15] = \yi[31] ;

  DFF_X1 \xbuf_real_stage2_reg[14]  ( .D(xbuf_real_stage1[14]), .CK(clk), .Q(
        xbuf_real_stage2[14]) );
  DFF_X1 \xbuf_real_stage2_reg[12]  ( .D(xbuf_real_stage1[12]), .CK(clk), .Q(
        xbuf_real_stage2[12]) );
  DFF_X1 \xbuf_real_stage2_reg[10]  ( .D(xbuf_real_stage1[10]), .CK(clk), .Q(
        xbuf_real_stage2[10]) );
  DFF_X1 \xbuf_real_stage2_reg[8]  ( .D(xbuf_real_stage1[8]), .CK(clk), .Q(
        xbuf_real_stage2[8]) );
  DFF_X1 \xbuf_real_stage2_reg[6]  ( .D(xbuf_real_stage1[6]), .CK(clk), .Q(
        xbuf_real_stage2[6]) );
  DFF_X1 \xbuf_real_stage2_reg[4]  ( .D(xbuf_real_stage1[4]), .CK(clk), .Q(
        xbuf_real_stage2[4]) );
  DFF_X1 \xbuf_real_stage2_reg[2]  ( .D(xbuf_real_stage1[2]), .CK(clk), .Q(
        xbuf_real_stage2[2]) );
  DFF_X1 \xbuf_real_stage2_reg[0]  ( .D(xbuf_real_stage1[0]), .CK(clk), .Q(
        xbuf_real_stage2[0]), .QN(n3) );
  DFF_X1 \xbuf_imag_stage2_reg[14]  ( .D(xbuf_imag_stage1[14]), .CK(clk), .Q(
        xbuf_imag_stage2[14]) );
  DFF_X1 \xbuf_imag_stage2_reg[12]  ( .D(xbuf_imag_stage1[12]), .CK(clk), .Q(
        xbuf_imag_stage2[12]) );
  DFF_X1 \xbuf_imag_stage2_reg[10]  ( .D(xbuf_imag_stage1[10]), .CK(clk), .Q(
        xbuf_imag_stage2[10]) );
  DFF_X1 \xbuf_imag_stage2_reg[8]  ( .D(xbuf_imag_stage1[8]), .CK(clk), .Q(
        xbuf_imag_stage2[8]) );
  DFF_X1 \xbuf_imag_stage2_reg[6]  ( .D(xbuf_imag_stage1[6]), .CK(clk), .Q(
        xbuf_imag_stage2[6]) );
  DFF_X1 \xbuf_imag_stage2_reg[4]  ( .D(xbuf_imag_stage1[4]), .CK(clk), .Q(
        xbuf_imag_stage2[4]) );
  DFF_X1 \xbuf_imag_stage2_reg[2]  ( .D(xbuf_imag_stage1[2]), .CK(clk), .Q(
        xbuf_imag_stage2[2]) );
  DFF_X1 \xbuf_imag_stage2_reg[0]  ( .D(xbuf_imag_stage1[0]), .CK(clk), .Q(
        xbuf_imag_stage2[0]), .QN(n2) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[15]  ( .D(xbuf_real_p_imag_stage1[15]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[15]), .QN(n2057) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[13]  ( .D(xbuf_real_p_imag_stage1[13]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[13]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[12]  ( .D(xbuf_real_p_imag_stage1[12]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[12]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[10]  ( .D(xbuf_real_p_imag_stage1[10]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[10]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[9]  ( .D(xbuf_real_p_imag_stage1[9]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[9]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[7]  ( .D(xbuf_real_p_imag_stage1[7]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[7]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[6]  ( .D(xbuf_real_p_imag_stage1[6]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[6]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[4]  ( .D(xbuf_real_p_imag_stage1[4]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[4]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[3]  ( .D(xbuf_real_p_imag_stage1[3]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[3]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[2]  ( .D(xbuf_real_p_imag_stage1[2]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[2]) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[1]  ( .D(xbuf_real_p_imag_stage1[1]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[1]), .QN(n2042) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[0]  ( .D(xbuf_real_p_imag_stage1[0]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[0]), .QN(n2044) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[17]  ( .D(
        twiddle_real_p_imag_stage1[17]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[17]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[16]  ( .D(
        twiddle_real_p_imag_stage1[16]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[16]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[15]  ( .D(
        twiddle_real_p_imag_stage1[15]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[15]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[14]  ( .D(
        twiddle_real_p_imag_stage1[14]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[14]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[13]  ( .D(
        twiddle_real_p_imag_stage1[13]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[13]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[12]  ( .D(
        twiddle_real_p_imag_stage1[12]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[12]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[11]  ( .D(
        twiddle_real_p_imag_stage1[11]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[11]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[10]  ( .D(
        twiddle_real_p_imag_stage1[10]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[10]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[9]  ( .D(
        twiddle_real_p_imag_stage1[9]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[9]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[8]  ( .D(
        twiddle_real_p_imag_stage1[8]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[8]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[7]  ( .D(
        twiddle_real_p_imag_stage1[7]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[7]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[6]  ( .D(
        twiddle_real_p_imag_stage1[6]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[6]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[5]  ( .D(
        twiddle_real_p_imag_stage1[5]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[5]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[4]  ( .D(
        twiddle_real_p_imag_stage1[4]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[4]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[3]  ( .D(
        twiddle_real_p_imag_stage1[3]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[3]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[2]  ( .D(
        twiddle_real_p_imag_stage1[2]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[2]) );
  DFF_X1 \twiddle_real_p_imag_stage2_reg[1]  ( .D(
        twiddle_real_p_imag_stage1[1]), .CK(clk), .Q(
        twiddle_real_p_imag_stage2[1]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[17]  ( .D(
        twiddle_real_m_imag_stage1[17]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[17]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[16]  ( .D(
        twiddle_real_m_imag_stage1[16]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[16]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[15]  ( .D(
        twiddle_real_m_imag_stage1[15]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[15]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[14]  ( .D(
        twiddle_real_m_imag_stage1[14]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[14]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[13]  ( .D(
        twiddle_real_m_imag_stage1[13]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[13]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[12]  ( .D(
        twiddle_real_m_imag_stage1[12]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[12]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[11]  ( .D(
        twiddle_real_m_imag_stage1[11]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[11]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[10]  ( .D(
        twiddle_real_m_imag_stage1[10]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[10]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[9]  ( .D(
        twiddle_real_m_imag_stage1[9]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[9]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[8]  ( .D(
        twiddle_real_m_imag_stage1[8]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[8]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[7]  ( .D(
        twiddle_real_m_imag_stage1[7]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[7]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[6]  ( .D(
        twiddle_real_m_imag_stage1[6]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[6]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[5]  ( .D(
        twiddle_real_m_imag_stage1[5]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[5]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[4]  ( .D(
        twiddle_real_m_imag_stage1[4]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[4]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[3]  ( .D(
        twiddle_real_m_imag_stage1[3]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[3]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[2]  ( .D(
        twiddle_real_m_imag_stage1[2]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[2]) );
  DFF_X1 \twiddle_real_m_imag_stage2_reg[1]  ( .D(
        twiddle_real_m_imag_stage1[1]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[1]) );
  DFF_X1 \twiddle_real_stage2_reg[2]  ( .D(twiddle_real[2]), .CK(clk), .Q(
        twiddle_real_stage2[2]) );
  DFF_X1 \tmp_a_stage3_reg[31]  ( .D(tmp_a[31]), .CK(clk), .Q(tmp_a_stage3[31]) );
  DFF_X1 \tmp_a_stage3_reg[30]  ( .D(tmp_a[30]), .CK(clk), .Q(tmp_a_stage3[30]) );
  DFF_X1 \tmp_a_stage3_reg[29]  ( .D(tmp_a[29]), .CK(clk), .Q(tmp_a_stage3[29]) );
  DFF_X1 \tmp_a_stage3_reg[28]  ( .D(tmp_a[28]), .CK(clk), .Q(tmp_a_stage3[28]) );
  DFF_X1 \tmp_a_stage3_reg[27]  ( .D(tmp_a[27]), .CK(clk), .Q(tmp_a_stage3[27]) );
  DFF_X1 \tmp_a_stage3_reg[26]  ( .D(tmp_a[26]), .CK(clk), .Q(tmp_a_stage3[26]) );
  DFF_X1 \tmp_a_stage3_reg[25]  ( .D(tmp_a[25]), .CK(clk), .Q(tmp_a_stage3[25]) );
  DFF_X1 \tmp_a_stage3_reg[24]  ( .D(tmp_a[24]), .CK(clk), .Q(tmp_a_stage3[24]) );
  DFF_X1 \tmp_a_stage3_reg[23]  ( .D(tmp_a[23]), .CK(clk), .Q(tmp_a_stage3[23]) );
  DFF_X1 \tmp_a_stage3_reg[22]  ( .D(tmp_a[22]), .CK(clk), .Q(tmp_a_stage3[22]) );
  DFF_X1 \tmp_a_stage3_reg[21]  ( .D(tmp_a[21]), .CK(clk), .Q(tmp_a_stage3[21]) );
  DFF_X1 \tmp_a_stage3_reg[20]  ( .D(tmp_a[20]), .CK(clk), .Q(tmp_a_stage3[20]) );
  DFF_X1 \tmp_a_stage3_reg[19]  ( .D(tmp_a[19]), .CK(clk), .Q(tmp_a_stage3[19]) );
  DFF_X1 \tmp_a_stage3_reg[18]  ( .D(tmp_a[18]), .CK(clk), .Q(tmp_a_stage3[18]) );
  DFF_X1 \tmp_a_stage3_reg[17]  ( .D(tmp_a[17]), .CK(clk), .Q(tmp_a_stage3[17]) );
  DFF_X1 \tmp_a_stage3_reg[16]  ( .D(tmp_a[16]), .CK(clk), .Q(tmp_a_stage3[16]) );
  DFF_X1 \tmp_a_stage3_reg[15]  ( .D(tmp_a[15]), .CK(clk), .Q(tmp_a_stage3[15]) );
  DFF_X1 \tmp_a_stage3_reg[14]  ( .D(tmp_a[14]), .CK(clk), .Q(tmp_a_stage3[14]) );
  DFF_X1 \tmp_a_stage3_reg[13]  ( .D(tmp_a[13]), .CK(clk), .Q(tmp_a_stage3[13]) );
  DFF_X1 \tmp_a_stage3_reg[12]  ( .D(tmp_a[12]), .CK(clk), .QN(n2064) );
  DFF_X1 \tmp_a_stage3_reg[11]  ( .D(tmp_a[11]), .CK(clk), .Q(tmp_a_stage3[11]) );
  DFF_X1 \tmp_a_stage3_reg[10]  ( .D(tmp_a[10]), .CK(clk), .QN(n2061) );
  DFF_X1 \tmp_a_stage3_reg[9]  ( .D(tmp_a[9]), .CK(clk), .Q(tmp_a_stage3[9])
         );
  DFF_X1 \tmp_a_stage3_reg[8]  ( .D(tmp_a[8]), .CK(clk), .QN(n2058) );
  DFF_X1 \tmp_a_stage3_reg[7]  ( .D(tmp_a[7]), .CK(clk), .Q(tmp_a_stage3[7])
         );
  DFF_X1 \tmp_a_stage3_reg[6]  ( .D(tmp_a[6]), .CK(clk), .QN(n2055) );
  DFF_X1 \tmp_a_stage3_reg[5]  ( .D(tmp_a[5]), .CK(clk), .Q(tmp_a_stage3[5])
         );
  DFF_X1 \tmp_a_stage3_reg[4]  ( .D(tmp_a[4]), .CK(clk), .QN(n2052) );
  DFF_X1 \tmp_a_stage3_reg[3]  ( .D(tmp_a[3]), .CK(clk), .Q(tmp_a_stage3[3])
         );
  DFF_X1 \tmp_a_stage3_reg[2]  ( .D(tmp_a[2]), .CK(clk), .QN(n2048) );
  DFF_X1 \tmp_a_stage3_reg[1]  ( .D(tmp_a[1]), .CK(clk), .Q(tmp_a_stage3[1]), 
        .QN(n2046) );
  DFF_X1 \tmp_a_stage3_reg[0]  ( .D(tmp_a[0]), .CK(clk), .Q(tmp_a_stage3[0])
         );
  DFF_X1 \tmp_r_stage3_reg[31]  ( .D(tmp_r[31]), .CK(clk), .Q(tmp_r_stage3[31]) );
  DFF_X1 \tmp_r_stage3_reg[30]  ( .D(tmp_r[30]), .CK(clk), .Q(tmp_r_stage3[30]) );
  DFF_X1 \tmp_r_stage3_reg[29]  ( .D(tmp_r[29]), .CK(clk), .Q(tmp_r_stage3[29]) );
  DFF_X1 \tmp_r_stage3_reg[28]  ( .D(tmp_r[28]), .CK(clk), .Q(tmp_r_stage3[28]) );
  DFF_X1 \tmp_r_stage3_reg[27]  ( .D(tmp_r[27]), .CK(clk), .Q(tmp_r_stage3[27]) );
  DFF_X1 \tmp_r_stage3_reg[26]  ( .D(tmp_r[26]), .CK(clk), .Q(tmp_r_stage3[26]) );
  DFF_X1 \tmp_r_stage3_reg[25]  ( .D(tmp_r[25]), .CK(clk), .Q(tmp_r_stage3[25]) );
  DFF_X1 \tmp_r_stage3_reg[24]  ( .D(tmp_r[24]), .CK(clk), .Q(tmp_r_stage3[24]) );
  DFF_X1 \tmp_r_stage3_reg[23]  ( .D(tmp_r[23]), .CK(clk), .Q(tmp_r_stage3[23]) );
  DFF_X1 \tmp_r_stage3_reg[22]  ( .D(tmp_r[22]), .CK(clk), .Q(tmp_r_stage3[22]) );
  DFF_X1 \tmp_r_stage3_reg[21]  ( .D(tmp_r[21]), .CK(clk), .Q(tmp_r_stage3[21]) );
  DFF_X1 \tmp_r_stage3_reg[20]  ( .D(tmp_r[20]), .CK(clk), .Q(tmp_r_stage3[20]) );
  DFF_X1 \tmp_r_stage3_reg[19]  ( .D(tmp_r[19]), .CK(clk), .Q(tmp_r_stage3[19]) );
  DFF_X1 \tmp_r_stage3_reg[18]  ( .D(tmp_r[18]), .CK(clk), .Q(tmp_r_stage3[18]) );
  DFF_X1 \tmp_r_stage3_reg[17]  ( .D(tmp_r[17]), .CK(clk), .Q(tmp_r_stage3[17]) );
  DFF_X1 \tmp_r_stage3_reg[16]  ( .D(tmp_r[16]), .CK(clk), .Q(tmp_r_stage3[16]) );
  DFF_X1 \tmp_r_stage3_reg[15]  ( .D(tmp_r[15]), .CK(clk), .QN(n2068) );
  DFF_X1 \tmp_r_stage3_reg[14]  ( .D(tmp_r[14]), .CK(clk), .Q(tmp_r_stage3[14]) );
  DFF_X1 \tmp_r_stage3_reg[13]  ( .D(tmp_r[13]), .CK(clk), .Q(tmp_r_stage3[13]) );
  DFF_X1 \tmp_r_stage3_reg[12]  ( .D(tmp_r[12]), .CK(clk), .Q(tmp_r_stage3[12]) );
  DFF_X1 \tmp_r_stage3_reg[11]  ( .D(tmp_r[11]), .CK(clk), .Q(tmp_r_stage3[11]) );
  DFF_X1 \tmp_r_stage3_reg[10]  ( .D(tmp_r[10]), .CK(clk), .Q(tmp_r_stage3[10]) );
  DFF_X1 \tmp_r_stage3_reg[9]  ( .D(tmp_r[9]), .CK(clk), .Q(tmp_r_stage3[9])
         );
  DFF_X1 \tmp_r_stage3_reg[8]  ( .D(tmp_r[8]), .CK(clk), .Q(tmp_r_stage3[8])
         );
  DFF_X1 \tmp_r_stage3_reg[7]  ( .D(tmp_r[7]), .CK(clk), .Q(tmp_r_stage3[7])
         );
  DFF_X1 \tmp_r_stage3_reg[6]  ( .D(tmp_r[6]), .CK(clk), .Q(tmp_r_stage3[6])
         );
  DFF_X1 \tmp_r_stage3_reg[5]  ( .D(tmp_r[5]), .CK(clk), .Q(tmp_r_stage3[5])
         );
  DFF_X1 \tmp_r_stage3_reg[4]  ( .D(tmp_r[4]), .CK(clk), .Q(tmp_r_stage3[4])
         );
  DFF_X1 \tmp_r_stage3_reg[3]  ( .D(tmp_r[3]), .CK(clk), .Q(tmp_r_stage3[3])
         );
  DFF_X1 \tmp_r_stage3_reg[2]  ( .D(tmp_r[2]), .CK(clk), .Q(tmp_r_stage3[2])
         );
  DFF_X1 \tmp_r_stage3_reg[1]  ( .D(n40), .CK(clk), .Q(tmp_r_stage3[1]), .QN(
        n2047) );
  DFF_X1 \tmp_r_stage3_reg[0]  ( .D(tmp_r[0]), .CK(clk), .Q(tmp_r_stage3[0])
         );
  DFF_X1 \tmp_i_stage3_reg[31]  ( .D(tmp_i[31]), .CK(clk), .Q(tmp_i_stage3[31]) );
  DFF_X1 \tmp_i_stage3_reg[30]  ( .D(tmp_i[30]), .CK(clk), .Q(tmp_i_stage3[30]) );
  DFF_X1 \tmp_i_stage3_reg[29]  ( .D(tmp_i[29]), .CK(clk), .Q(tmp_i_stage3[29]) );
  DFF_X1 \tmp_i_stage3_reg[28]  ( .D(tmp_i[28]), .CK(clk), .Q(tmp_i_stage3[28]) );
  DFF_X1 \tmp_i_stage3_reg[27]  ( .D(tmp_i[27]), .CK(clk), .Q(tmp_i_stage3[27]) );
  DFF_X1 \tmp_i_stage3_reg[26]  ( .D(tmp_i[26]), .CK(clk), .Q(tmp_i_stage3[26]) );
  DFF_X1 \tmp_i_stage3_reg[25]  ( .D(tmp_i[25]), .CK(clk), .Q(tmp_i_stage3[25]) );
  DFF_X1 \tmp_i_stage3_reg[24]  ( .D(tmp_i[24]), .CK(clk), .Q(tmp_i_stage3[24]) );
  DFF_X1 \tmp_i_stage3_reg[23]  ( .D(tmp_i[23]), .CK(clk), .Q(tmp_i_stage3[23]) );
  DFF_X1 \tmp_i_stage3_reg[22]  ( .D(tmp_i[22]), .CK(clk), .Q(tmp_i_stage3[22]) );
  DFF_X1 \tmp_i_stage3_reg[21]  ( .D(tmp_i[21]), .CK(clk), .Q(tmp_i_stage3[21]) );
  DFF_X1 \tmp_i_stage3_reg[20]  ( .D(tmp_i[20]), .CK(clk), .Q(tmp_i_stage3[20]) );
  DFF_X1 \tmp_i_stage3_reg[19]  ( .D(tmp_i[19]), .CK(clk), .Q(tmp_i_stage3[19]) );
  DFF_X1 \tmp_i_stage3_reg[18]  ( .D(tmp_i[18]), .CK(clk), .Q(tmp_i_stage3[18]) );
  DFF_X1 \tmp_i_stage3_reg[17]  ( .D(tmp_i[17]), .CK(clk), .Q(tmp_i_stage3[17]) );
  DFF_X1 \tmp_i_stage3_reg[16]  ( .D(tmp_i[16]), .CK(clk), .Q(tmp_i_stage3[16]) );
  DFF_X1 \tmp_i_stage3_reg[15]  ( .D(tmp_i[15]), .CK(clk), .QN(n2067) );
  DFF_X1 \tmp_i_stage3_reg[14]  ( .D(tmp_i[14]), .CK(clk), .Q(tmp_i_stage3[14]) );
  DFF_X1 \tmp_i_stage3_reg[13]  ( .D(tmp_i[13]), .CK(clk), .QN(n2065) );
  DFF_X1 \tmp_i_stage3_reg[12]  ( .D(tmp_i[12]), .CK(clk), .Q(tmp_i_stage3[12]) );
  DFF_X1 \tmp_i_stage3_reg[11]  ( .D(tmp_i[11]), .CK(clk), .QN(n2063) );
  DFF_X1 \tmp_i_stage3_reg[10]  ( .D(tmp_i[10]), .CK(clk), .Q(tmp_i_stage3[10]) );
  DFF_X1 \tmp_i_stage3_reg[9]  ( .D(tmp_i[9]), .CK(clk), .QN(n2060) );
  DFF_X1 \tmp_i_stage3_reg[8]  ( .D(tmp_i[8]), .CK(clk), .Q(tmp_i_stage3[8])
         );
  DFF_X1 \tmp_i_stage3_reg[7]  ( .D(tmp_i[7]), .CK(clk), .QN(n2059) );
  DFF_X1 \tmp_i_stage3_reg[6]  ( .D(tmp_i[6]), .CK(clk), .Q(tmp_i_stage3[6])
         );
  DFF_X1 \tmp_i_stage3_reg[5]  ( .D(tmp_i[5]), .CK(clk), .QN(n2056) );
  DFF_X1 \tmp_i_stage3_reg[4]  ( .D(tmp_i[4]), .CK(clk), .Q(tmp_i_stage3[4])
         );
  DFF_X1 \tmp_i_stage3_reg[3]  ( .D(tmp_i[3]), .CK(clk), .QN(n2054) );
  DFF_X1 \tmp_i_stage3_reg[2]  ( .D(tmp_i[2]), .CK(clk), .Q(tmp_i_stage3[2])
         );
  DFF_X1 \tmp_i_stage3_reg[1]  ( .D(n8), .CK(clk), .Q(tmp_i_stage3[1]), .QN(
        n2051) );
  DFF_X1 \tmp_i_stage3_reg[0]  ( .D(tmp_i[0]), .CK(clk), .Q(tmp_i_stage3[0])
         );
  DFF_X1 \dst_opa_real_stage2_reg[15]  ( .D(N93), .CK(clk), .Q(
        dst_opa_real_stage2[15]) );
  DFF_X1 \dst_opa_real_stage2_reg[14]  ( .D(N92), .CK(clk), .Q(
        dst_opa_real_stage2[14]) );
  DFF_X1 \dst_opa_real_stage2_reg[13]  ( .D(N91), .CK(clk), .Q(
        dst_opa_real_stage2[13]) );
  DFF_X1 \dst_opa_real_stage2_reg[12]  ( .D(N90), .CK(clk), .Q(
        dst_opa_real_stage2[12]) );
  DFF_X1 \dst_opa_real_stage2_reg[11]  ( .D(N89), .CK(clk), .Q(
        dst_opa_real_stage2[11]) );
  DFF_X1 \dst_opa_real_stage2_reg[10]  ( .D(N88), .CK(clk), .Q(
        dst_opa_real_stage2[10]) );
  DFF_X1 \dst_opa_real_stage2_reg[9]  ( .D(N87), .CK(clk), .Q(
        dst_opa_real_stage2[9]) );
  DFF_X1 \dst_opa_real_stage2_reg[8]  ( .D(N86), .CK(clk), .Q(
        dst_opa_real_stage2[8]) );
  DFF_X1 \dst_opa_real_stage2_reg[7]  ( .D(N85), .CK(clk), .Q(
        dst_opa_real_stage2[7]) );
  DFF_X1 \dst_opa_real_stage2_reg[6]  ( .D(N84), .CK(clk), .Q(
        dst_opa_real_stage2[6]) );
  DFF_X1 \dst_opa_real_stage2_reg[5]  ( .D(N83), .CK(clk), .Q(
        dst_opa_real_stage2[5]) );
  DFF_X1 \dst_opa_real_stage2_reg[4]  ( .D(N82), .CK(clk), .Q(
        dst_opa_real_stage2[4]) );
  DFF_X1 \dst_opa_real_stage2_reg[3]  ( .D(N81), .CK(clk), .Q(
        dst_opa_real_stage2[3]) );
  DFF_X1 \dst_opa_real_stage2_reg[2]  ( .D(N80), .CK(clk), .Q(
        dst_opa_real_stage2[2]) );
  DFF_X1 \dst_opa_real_stage2_reg[1]  ( .D(N79), .CK(clk), .Q(
        dst_opa_real_stage2[1]) );
  DFF_X1 \dst_opa_real_stage2_reg[0]  ( .D(N78), .CK(clk), .Q(
        dst_opa_real_stage2[0]) );
  DFF_X1 \dst_opa_imag_stage2_reg[15]  ( .D(N109), .CK(clk), .Q(
        dst_opa_imag_stage2[15]) );
  DFF_X1 \dst_opa_imag_stage2_reg[14]  ( .D(N108), .CK(clk), .Q(
        dst_opa_imag_stage2[14]) );
  DFF_X1 \dst_opa_imag_stage2_reg[13]  ( .D(N107), .CK(clk), .Q(
        dst_opa_imag_stage2[13]) );
  DFF_X1 \dst_opa_imag_stage2_reg[12]  ( .D(N106), .CK(clk), .Q(
        dst_opa_imag_stage2[12]) );
  DFF_X1 \dst_opa_imag_stage2_reg[11]  ( .D(N105), .CK(clk), .Q(
        dst_opa_imag_stage2[11]) );
  DFF_X1 \dst_opa_imag_stage2_reg[10]  ( .D(N104), .CK(clk), .Q(
        dst_opa_imag_stage2[10]) );
  DFF_X1 \dst_opa_imag_stage2_reg[9]  ( .D(N103), .CK(clk), .Q(
        dst_opa_imag_stage2[9]) );
  DFF_X1 \dst_opa_imag_stage2_reg[8]  ( .D(N102), .CK(clk), .Q(
        dst_opa_imag_stage2[8]) );
  DFF_X1 \dst_opa_imag_stage2_reg[7]  ( .D(N101), .CK(clk), .Q(
        dst_opa_imag_stage2[7]) );
  DFF_X1 \dst_opa_imag_stage2_reg[6]  ( .D(N100), .CK(clk), .Q(
        dst_opa_imag_stage2[6]) );
  DFF_X1 \dst_opa_imag_stage2_reg[5]  ( .D(N99), .CK(clk), .Q(
        dst_opa_imag_stage2[5]) );
  DFF_X1 \dst_opa_imag_stage2_reg[4]  ( .D(N98), .CK(clk), .Q(
        dst_opa_imag_stage2[4]) );
  DFF_X1 \dst_opa_imag_stage2_reg[3]  ( .D(N97), .CK(clk), .Q(
        dst_opa_imag_stage2[3]) );
  DFF_X1 \dst_opa_imag_stage2_reg[2]  ( .D(N96), .CK(clk), .Q(
        dst_opa_imag_stage2[2]) );
  DFF_X1 \dst_opa_imag_stage2_reg[1]  ( .D(N95), .CK(clk), .Q(
        dst_opa_imag_stage2[1]) );
  DFF_X1 \dst_opa_imag_stage2_reg[0]  ( .D(N94), .CK(clk), .Q(
        dst_opa_imag_stage2[0]) );
  DFF_X1 iact_stage2_reg ( .D(N111), .CK(clk), .QN(n2070) );
  DFF_X1 \ictrl_stage2_reg[1]  ( .D(ictrl[1]), .CK(clk), .Q(ictrl_stage2[1])
         );
  DFF_X1 \ictrl_stage2_reg[0]  ( .D(ictrl[0]), .CK(clk), .Q(ictrl_stage2[0])
         );
  DFF_X1 \iMemAddr_stage2_reg[8]  ( .D(iMemAddr[8]), .CK(clk), .Q(
        iMemAddr_stage2[8]) );
  DFF_X1 \iMemAddr_stage2_reg[7]  ( .D(iMemAddr[7]), .CK(clk), .Q(
        iMemAddr_stage2[7]) );
  DFF_X1 \iMemAddr_stage2_reg[6]  ( .D(iMemAddr[6]), .CK(clk), .Q(
        iMemAddr_stage2[6]) );
  DFF_X1 \iMemAddr_stage2_reg[5]  ( .D(iMemAddr[5]), .CK(clk), .Q(
        iMemAddr_stage2[5]) );
  DFF_X1 \iMemAddr_stage2_reg[4]  ( .D(iMemAddr[4]), .CK(clk), .Q(
        iMemAddr_stage2[4]) );
  DFF_X1 \iMemAddr_stage2_reg[3]  ( .D(iMemAddr[3]), .CK(clk), .Q(
        iMemAddr_stage2[3]) );
  DFF_X1 \iMemAddr_stage2_reg[2]  ( .D(iMemAddr[2]), .CK(clk), .Q(
        iMemAddr_stage2[2]) );
  DFF_X1 \iMemAddr_stage2_reg[1]  ( .D(iMemAddr[1]), .CK(clk), .Q(
        iMemAddr_stage2[1]) );
  DFF_X1 \iMemAddr_stage2_reg[0]  ( .D(iMemAddr[0]), .CK(clk), .Q(
        iMemAddr_stage2[0]) );
  DFF_X1 \dst_opa_real_reg[15]  ( .D(dst_opa_real_stage2[15]), .CK(clk), .Q(
        dst_opa_real[15]) );
  DFF_X1 \dst_opa_real_reg[14]  ( .D(dst_opa_real_stage2[14]), .CK(clk), .Q(
        dst_opa_real[14]) );
  DFF_X1 \dst_opa_real_reg[13]  ( .D(dst_opa_real_stage2[13]), .CK(clk), .Q(
        dst_opa_real[13]) );
  DFF_X1 \dst_opa_real_reg[12]  ( .D(dst_opa_real_stage2[12]), .CK(clk), .Q(
        dst_opa_real[12]) );
  DFF_X1 \dst_opa_real_reg[11]  ( .D(dst_opa_real_stage2[11]), .CK(clk), .Q(
        dst_opa_real[11]) );
  DFF_X1 \dst_opa_real_reg[10]  ( .D(dst_opa_real_stage2[10]), .CK(clk), .Q(
        dst_opa_real[10]) );
  DFF_X1 \dst_opa_real_reg[9]  ( .D(dst_opa_real_stage2[9]), .CK(clk), .Q(
        dst_opa_real[9]) );
  DFF_X1 \dst_opa_real_reg[8]  ( .D(dst_opa_real_stage2[8]), .CK(clk), .Q(
        dst_opa_real[8]) );
  DFF_X1 \dst_opa_real_reg[7]  ( .D(dst_opa_real_stage2[7]), .CK(clk), .Q(
        dst_opa_real[7]) );
  DFF_X1 \dst_opa_real_reg[6]  ( .D(dst_opa_real_stage2[6]), .CK(clk), .Q(
        dst_opa_real[6]) );
  DFF_X1 \dst_opa_real_reg[5]  ( .D(dst_opa_real_stage2[5]), .CK(clk), .Q(
        dst_opa_real[5]) );
  DFF_X1 \dst_opa_real_reg[4]  ( .D(dst_opa_real_stage2[4]), .CK(clk), .Q(
        dst_opa_real[4]) );
  DFF_X1 \dst_opa_real_reg[3]  ( .D(dst_opa_real_stage2[3]), .CK(clk), .Q(
        dst_opa_real[3]) );
  DFF_X1 \dst_opa_real_reg[2]  ( .D(dst_opa_real_stage2[2]), .CK(clk), .Q(
        dst_opa_real[2]) );
  DFF_X1 \dst_opa_real_reg[1]  ( .D(dst_opa_real_stage2[1]), .CK(clk), .Q(
        dst_opa_real[1]) );
  DFF_X1 \dst_opa_real_reg[0]  ( .D(dst_opa_real_stage2[0]), .CK(clk), .Q(
        dst_opa_real[0]) );
  DFF_X1 \dst_opa_imag_reg[15]  ( .D(dst_opa_imag_stage2[15]), .CK(clk), .Q(
        dst_opa_imag[15]) );
  DFF_X1 \dst_opa_imag_reg[14]  ( .D(dst_opa_imag_stage2[14]), .CK(clk), .Q(
        dst_opa_imag[14]) );
  DFF_X1 \dst_opa_imag_reg[13]  ( .D(dst_opa_imag_stage2[13]), .CK(clk), .Q(
        dst_opa_imag[13]) );
  DFF_X1 \dst_opa_imag_reg[12]  ( .D(dst_opa_imag_stage2[12]), .CK(clk), .Q(
        dst_opa_imag[12]) );
  DFF_X1 \dst_opa_imag_reg[11]  ( .D(dst_opa_imag_stage2[11]), .CK(clk), .Q(
        dst_opa_imag[11]) );
  DFF_X1 \dst_opa_imag_reg[10]  ( .D(dst_opa_imag_stage2[10]), .CK(clk), .Q(
        dst_opa_imag[10]) );
  DFF_X1 \dst_opa_imag_reg[9]  ( .D(dst_opa_imag_stage2[9]), .CK(clk), .Q(
        dst_opa_imag[9]) );
  DFF_X1 \dst_opa_imag_reg[8]  ( .D(dst_opa_imag_stage2[8]), .CK(clk), .Q(
        dst_opa_imag[8]) );
  DFF_X1 \dst_opa_imag_reg[7]  ( .D(dst_opa_imag_stage2[7]), .CK(clk), .Q(
        dst_opa_imag[7]) );
  DFF_X1 \dst_opa_imag_reg[6]  ( .D(dst_opa_imag_stage2[6]), .CK(clk), .Q(
        dst_opa_imag[6]) );
  DFF_X1 \dst_opa_imag_reg[5]  ( .D(dst_opa_imag_stage2[5]), .CK(clk), .Q(
        dst_opa_imag[5]) );
  DFF_X1 \dst_opa_imag_reg[4]  ( .D(dst_opa_imag_stage2[4]), .CK(clk), .Q(
        dst_opa_imag[4]) );
  DFF_X1 \dst_opa_imag_reg[3]  ( .D(dst_opa_imag_stage2[3]), .CK(clk), .Q(
        dst_opa_imag[3]) );
  DFF_X1 \dst_opa_imag_reg[2]  ( .D(dst_opa_imag_stage2[2]), .CK(clk), .Q(
        dst_opa_imag[2]) );
  DFF_X1 \dst_opa_imag_reg[1]  ( .D(dst_opa_imag_stage2[1]), .CK(clk), .Q(
        dst_opa_imag[1]) );
  DFF_X1 \dst_opa_imag_reg[0]  ( .D(dst_opa_imag_stage2[0]), .CK(clk), .Q(
        dst_opa_imag[0]) );
  DFF_X1 oact_reg ( .D(N112), .CK(clk), .Q(oact) );
  DFF_X1 \octrl_reg[1]  ( .D(ictrl_stage2[1]), .CK(clk), .Q(octrl[1]) );
  DFF_X1 \octrl_reg[0]  ( .D(ictrl_stage2[0]), .CK(clk), .Q(octrl[0]) );
  DFF_X1 \oMemAddr_reg[8]  ( .D(iMemAddr_stage2[8]), .CK(clk), .Q(oMemAddr[8])
         );
  DFF_X1 \oMemAddr_reg[7]  ( .D(iMemAddr_stage2[7]), .CK(clk), .Q(oMemAddr[7])
         );
  DFF_X1 \oMemAddr_reg[6]  ( .D(iMemAddr_stage2[6]), .CK(clk), .Q(oMemAddr[6])
         );
  DFF_X1 \oMemAddr_reg[5]  ( .D(iMemAddr_stage2[5]), .CK(clk), .Q(oMemAddr[5])
         );
  DFF_X1 \oMemAddr_reg[4]  ( .D(iMemAddr_stage2[4]), .CK(clk), .Q(oMemAddr[4])
         );
  DFF_X1 \oMemAddr_reg[3]  ( .D(iMemAddr_stage2[3]), .CK(clk), .Q(oMemAddr[3])
         );
  DFF_X1 \oMemAddr_reg[2]  ( .D(iMemAddr_stage2[2]), .CK(clk), .Q(oMemAddr[2])
         );
  DFF_X1 \oMemAddr_reg[1]  ( .D(iMemAddr_stage2[1]), .CK(clk), .Q(oMemAddr[1])
         );
  DFF_X1 \oMemAddr_reg[0]  ( .D(iMemAddr_stage2[0]), .CK(clk), .Q(oMemAddr[0])
         );
  FA_X1 \intadd_1/U16  ( .A(tmp_r_stage3[16]), .B(\intadd_1/B[0] ), .CI(
        \intadd_1/CI ), .CO(\intadd_1/n15 ), .S(\intadd_1/SUM[0] ) );
  FA_X1 \intadd_1/U15  ( .A(tmp_r_stage3[17]), .B(\intadd_1/B[1] ), .CI(
        \intadd_1/n15 ), .CO(\intadd_1/n14 ), .S(\intadd_1/SUM[1] ) );
  FA_X1 \intadd_1/U14  ( .A(tmp_r_stage3[18]), .B(\intadd_1/B[2] ), .CI(
        \intadd_1/n14 ), .CO(\intadd_1/n13 ), .S(\intadd_1/SUM[2] ) );
  FA_X1 \intadd_1/U13  ( .A(tmp_r_stage3[19]), .B(\intadd_1/B[3] ), .CI(
        \intadd_1/n13 ), .CO(\intadd_1/n12 ), .S(\intadd_1/SUM[3] ) );
  FA_X1 \intadd_1/U12  ( .A(tmp_r_stage3[20]), .B(\intadd_1/B[4] ), .CI(
        \intadd_1/n12 ), .CO(\intadd_1/n11 ), .S(\intadd_1/SUM[4] ) );
  FA_X1 \intadd_1/U11  ( .A(tmp_r_stage3[21]), .B(\intadd_1/B[5] ), .CI(
        \intadd_1/n11 ), .CO(\intadd_1/n10 ), .S(\intadd_1/SUM[5] ) );
  FA_X1 \intadd_1/U10  ( .A(tmp_r_stage3[22]), .B(\intadd_1/B[6] ), .CI(
        \intadd_1/n10 ), .CO(\intadd_1/n9 ), .S(\intadd_1/SUM[6] ) );
  FA_X1 \intadd_1/U9  ( .A(tmp_r_stage3[23]), .B(\intadd_1/B[7] ), .CI(
        \intadd_1/n9 ), .CO(\intadd_1/n8 ), .S(\intadd_1/SUM[7] ) );
  FA_X1 \intadd_1/U8  ( .A(tmp_r_stage3[24]), .B(\intadd_1/B[8] ), .CI(
        \intadd_1/n8 ), .CO(\intadd_1/n7 ), .S(\intadd_1/SUM[8] ) );
  FA_X1 \intadd_1/U7  ( .A(tmp_r_stage3[25]), .B(\intadd_1/B[9] ), .CI(
        \intadd_1/n7 ), .CO(\intadd_1/n6 ), .S(\intadd_1/SUM[9] ) );
  FA_X1 \intadd_1/U6  ( .A(tmp_r_stage3[26]), .B(\intadd_1/B[10] ), .CI(
        \intadd_1/n6 ), .CO(\intadd_1/n5 ), .S(\intadd_1/SUM[10] ) );
  FA_X1 \intadd_1/U5  ( .A(tmp_r_stage3[27]), .B(\intadd_1/B[11] ), .CI(
        \intadd_1/n5 ), .CO(\intadd_1/n4 ), .S(\intadd_1/SUM[11] ) );
  FA_X1 \intadd_1/U4  ( .A(tmp_r_stage3[28]), .B(\intadd_1/B[12] ), .CI(
        \intadd_1/n4 ), .CO(\intadd_1/n3 ), .S(\intadd_1/SUM[12] ) );
  FA_X1 \intadd_1/U3  ( .A(tmp_r_stage3[29]), .B(\intadd_1/B[13] ), .CI(
        \intadd_1/n3 ), .CO(\intadd_1/n2 ), .S(\intadd_1/SUM[13] ) );
  FA_X1 \intadd_1/U2  ( .A(tmp_r_stage3[30]), .B(\intadd_1/B[14] ), .CI(
        \intadd_1/n2 ), .CO(\intadd_1/n1 ), .S(\intadd_1/SUM[14] ) );
  FA_X1 \intadd_2/U16  ( .A(tmp_i_stage3[16]), .B(\intadd_1/CI ), .CI(
        \intadd_2/CI ), .CO(\intadd_2/n15 ), .S(\intadd_2/SUM[0] ) );
  FA_X1 \intadd_2/U15  ( .A(tmp_i_stage3[17]), .B(\intadd_1/B[1] ), .CI(
        \intadd_2/n15 ), .CO(\intadd_2/n14 ), .S(\intadd_2/SUM[1] ) );
  FA_X1 \intadd_2/U14  ( .A(tmp_i_stage3[18]), .B(\intadd_1/B[2] ), .CI(
        \intadd_2/n14 ), .CO(\intadd_2/n13 ), .S(\intadd_2/SUM[2] ) );
  FA_X1 \intadd_2/U13  ( .A(tmp_i_stage3[19]), .B(\intadd_1/B[3] ), .CI(
        \intadd_2/n13 ), .CO(\intadd_2/n12 ), .S(\intadd_2/SUM[3] ) );
  FA_X1 \intadd_2/U12  ( .A(tmp_i_stage3[20]), .B(\intadd_1/B[4] ), .CI(
        \intadd_2/n12 ), .CO(\intadd_2/n11 ), .S(\intadd_2/SUM[4] ) );
  FA_X1 \intadd_2/U11  ( .A(tmp_i_stage3[21]), .B(\intadd_1/B[5] ), .CI(
        \intadd_2/n11 ), .CO(\intadd_2/n10 ), .S(\intadd_2/SUM[5] ) );
  FA_X1 \intadd_2/U10  ( .A(tmp_i_stage3[22]), .B(\intadd_1/B[6] ), .CI(
        \intadd_2/n10 ), .CO(\intadd_2/n9 ), .S(\intadd_2/SUM[6] ) );
  FA_X1 \intadd_2/U9  ( .A(tmp_i_stage3[23]), .B(\intadd_1/B[7] ), .CI(
        \intadd_2/n9 ), .CO(\intadd_2/n8 ), .S(\intadd_2/SUM[7] ) );
  FA_X1 \intadd_2/U8  ( .A(tmp_i_stage3[24]), .B(\intadd_1/B[8] ), .CI(
        \intadd_2/n8 ), .CO(\intadd_2/n7 ), .S(\intadd_2/SUM[8] ) );
  FA_X1 \intadd_2/U7  ( .A(tmp_i_stage3[25]), .B(\intadd_1/B[9] ), .CI(
        \intadd_2/n7 ), .CO(\intadd_2/n6 ), .S(\intadd_2/SUM[9] ) );
  FA_X1 \intadd_2/U6  ( .A(tmp_i_stage3[26]), .B(\intadd_1/B[10] ), .CI(
        \intadd_2/n6 ), .CO(\intadd_2/n5 ), .S(\intadd_2/SUM[10] ) );
  FA_X1 \intadd_2/U5  ( .A(tmp_i_stage3[27]), .B(\intadd_1/B[11] ), .CI(
        \intadd_2/n5 ), .CO(\intadd_2/n4 ), .S(\intadd_2/SUM[11] ) );
  FA_X1 \intadd_2/U4  ( .A(tmp_i_stage3[28]), .B(\intadd_1/B[12] ), .CI(
        \intadd_2/n4 ), .CO(\intadd_2/n3 ), .S(\intadd_2/SUM[12] ) );
  FA_X1 \intadd_2/U3  ( .A(tmp_i_stage3[29]), .B(\intadd_1/B[13] ), .CI(
        \intadd_2/n3 ), .CO(\intadd_2/n2 ), .S(\intadd_2/SUM[13] ) );
  FA_X1 \intadd_2/U2  ( .A(tmp_i_stage3[30]), .B(\intadd_1/B[14] ), .CI(
        \intadd_2/n2 ), .CO(\intadd_2/n1 ), .S(\intadd_2/SUM[14] ) );
  FA_X1 \intadd_3/U16  ( .A(xbuf_real_stage1[1]), .B(xbuf_imag_stage1[1]), 
        .CI(\intadd_3/CI ), .CO(\intadd_3/n15 ), .S(xbuf_real_p_imag_stage1[1]) );
  FA_X1 \intadd_3/U15  ( .A(xbuf_imag_stage1[2]), .B(xbuf_real_stage1[2]), 
        .CI(\intadd_3/n15 ), .CO(\intadd_3/n14 ), .S(
        xbuf_real_p_imag_stage1[2]) );
  FA_X1 \intadd_3/U14  ( .A(xbuf_imag_stage1[3]), .B(xbuf_real_stage1[3]), 
        .CI(\intadd_3/n14 ), .CO(\intadd_3/n13 ), .S(
        xbuf_real_p_imag_stage1[3]) );
  FA_X1 \intadd_3/U13  ( .A(xbuf_imag_stage1[4]), .B(xbuf_real_stage1[4]), 
        .CI(\intadd_3/n13 ), .CO(\intadd_3/n12 ), .S(
        xbuf_real_p_imag_stage1[4]) );
  FA_X1 \intadd_3/U12  ( .A(xbuf_imag_stage1[5]), .B(xbuf_real_stage1[5]), 
        .CI(\intadd_3/n12 ), .CO(\intadd_3/n11 ), .S(
        xbuf_real_p_imag_stage1[5]) );
  FA_X1 \intadd_3/U11  ( .A(xbuf_imag_stage1[6]), .B(xbuf_real_stage1[6]), 
        .CI(\intadd_3/n11 ), .CO(\intadd_3/n10 ), .S(
        xbuf_real_p_imag_stage1[6]) );
  FA_X1 \intadd_3/U10  ( .A(xbuf_imag_stage1[7]), .B(xbuf_real_stage1[7]), 
        .CI(\intadd_3/n10 ), .CO(\intadd_3/n9 ), .S(xbuf_real_p_imag_stage1[7]) );
  FA_X1 \intadd_3/U9  ( .A(xbuf_imag_stage1[8]), .B(xbuf_real_stage1[8]), .CI(
        \intadd_3/n9 ), .CO(\intadd_3/n8 ), .S(xbuf_real_p_imag_stage1[8]) );
  FA_X1 \intadd_3/U8  ( .A(xbuf_imag_stage1[9]), .B(xbuf_real_stage1[9]), .CI(
        \intadd_3/n8 ), .CO(\intadd_3/n7 ), .S(xbuf_real_p_imag_stage1[9]) );
  FA_X1 \intadd_3/U7  ( .A(xbuf_imag_stage1[10]), .B(xbuf_real_stage1[10]), 
        .CI(\intadd_3/n7 ), .CO(\intadd_3/n6 ), .S(xbuf_real_p_imag_stage1[10]) );
  FA_X1 \intadd_3/U6  ( .A(xbuf_imag_stage1[11]), .B(xbuf_real_stage1[11]), 
        .CI(\intadd_3/n6 ), .CO(\intadd_3/n5 ), .S(xbuf_real_p_imag_stage1[11]) );
  FA_X1 \intadd_3/U5  ( .A(xbuf_imag_stage1[12]), .B(xbuf_real_stage1[12]), 
        .CI(\intadd_3/n5 ), .CO(\intadd_3/n4 ), .S(xbuf_real_p_imag_stage1[12]) );
  FA_X1 \intadd_3/U4  ( .A(xbuf_imag_stage1[13]), .B(xbuf_real_stage1[13]), 
        .CI(\intadd_3/n4 ), .CO(\intadd_3/n3 ), .S(xbuf_real_p_imag_stage1[13]) );
  FA_X1 \intadd_3/U3  ( .A(xbuf_imag_stage1[14]), .B(xbuf_real_stage1[14]), 
        .CI(\intadd_3/n3 ), .CO(\intadd_3/n2 ), .S(xbuf_real_p_imag_stage1[14]) );
  FA_X1 \intadd_3/U2  ( .A(\intadd_3/A[14] ), .B(\intadd_3/B[14] ), .CI(
        \intadd_3/n2 ), .CO(\intadd_3/n1 ), .S(xbuf_real_p_imag_stage1[15]) );
  FA_X1 \intadd_4/U15  ( .A(twiddle_real_stage2[2]), .B(twiddle_real_stage2[3]), .CI(\intadd_4/CI ), .CO(\intadd_4/n14 ), .S(\intadd_4/SUM[0] ) );
  FA_X1 \intadd_4/U14  ( .A(twiddle_real_stage2[3]), .B(twiddle_real_stage2[4]), .CI(\intadd_4/n14 ), .CO(\intadd_4/n13 ), .S(\intadd_4/SUM[1] ) );
  FA_X1 \intadd_4/U13  ( .A(twiddle_real_stage2[4]), .B(twiddle_real_stage2[5]), .CI(\intadd_4/n13 ), .CO(\intadd_4/n12 ), .S(\intadd_4/SUM[2] ) );
  FA_X1 \intadd_4/U12  ( .A(twiddle_real_stage2[5]), .B(twiddle_real_stage2[6]), .CI(\intadd_4/n12 ), .CO(\intadd_4/n11 ), .S(\intadd_4/SUM[3] ) );
  FA_X1 \intadd_4/U11  ( .A(twiddle_real_stage2[6]), .B(twiddle_real_stage2[7]), .CI(\intadd_4/n11 ), .CO(\intadd_4/n10 ), .S(\intadd_4/SUM[4] ) );
  FA_X1 \intadd_4/U10  ( .A(twiddle_real_stage2[7]), .B(twiddle_real_stage2[8]), .CI(\intadd_4/n10 ), .CO(\intadd_4/n9 ), .S(\intadd_4/SUM[5] ) );
  FA_X1 \intadd_4/U9  ( .A(twiddle_real_stage2[8]), .B(twiddle_real_stage2[9]), 
        .CI(\intadd_4/n9 ), .CO(\intadd_4/n8 ), .S(\intadd_4/SUM[6] ) );
  FA_X1 \intadd_4/U8  ( .A(twiddle_real_stage2[9]), .B(twiddle_real_stage2[10]), .CI(\intadd_4/n8 ), .CO(\intadd_4/n7 ), .S(\intadd_4/SUM[7] ) );
  FA_X1 \intadd_4/U7  ( .A(twiddle_real_stage2[10]), .B(
        twiddle_real_stage2[11]), .CI(\intadd_4/n7 ), .CO(\intadd_4/n6 ), .S(
        \intadd_4/SUM[8] ) );
  FA_X1 \intadd_4/U6  ( .A(twiddle_real_stage2[11]), .B(
        twiddle_real_stage2[12]), .CI(\intadd_4/n6 ), .CO(\intadd_4/n5 ), .S(
        \intadd_4/SUM[9] ) );
  FA_X1 \intadd_4/U5  ( .A(twiddle_real_stage2[12]), .B(
        twiddle_real_stage2[13]), .CI(\intadd_4/n5 ), .CO(\intadd_4/n4 ), .S(
        \intadd_4/SUM[10] ) );
  FA_X1 \intadd_4/U4  ( .A(twiddle_real_stage2[13]), .B(
        twiddle_real_stage2[14]), .CI(\intadd_4/n4 ), .CO(\intadd_4/n3 ), .S(
        \intadd_4/SUM[11] ) );
  FA_X1 \intadd_4/U3  ( .A(twiddle_real_stage2[15]), .B(
        twiddle_real_stage2[14]), .CI(\intadd_4/n3 ), .CO(\intadd_4/n2 ), .S(
        \intadd_4/SUM[12] ) );
  FA_X1 \intadd_4/U2  ( .A(twiddle_real_stage2[15]), .B(
        twiddle_real_stage2[16]), .CI(\intadd_4/n2 ), .CO(\intadd_4/n1 ), .S(
        \intadd_4/SUM[13] ) );
  FA_X1 \intadd_5/U12  ( .A(\intadd_5/A[0] ), .B(\intadd_5/B[0] ), .CI(
        \intadd_5/CI ), .CO(\intadd_5/n11 ), .S(\intadd_5/SUM[0] ) );
  FA_X1 \intadd_5/U11  ( .A(\intadd_5/A[1] ), .B(\intadd_5/B[1] ), .CI(
        \intadd_5/n11 ), .CO(\intadd_5/n10 ), .S(\intadd_5/SUM[1] ) );
  FA_X1 \intadd_5/U10  ( .A(\intadd_5/A[2] ), .B(\intadd_5/B[2] ), .CI(
        \intadd_5/n10 ), .CO(\intadd_5/n9 ), .S(\intadd_5/SUM[2] ) );
  FA_X1 \intadd_5/U9  ( .A(\intadd_5/A[3] ), .B(\intadd_5/B[3] ), .CI(
        \intadd_5/n9 ), .CO(\intadd_5/n8 ), .S(\intadd_5/SUM[3] ) );
  FA_X1 \intadd_5/U8  ( .A(\intadd_5/A[4] ), .B(\intadd_5/B[4] ), .CI(
        \intadd_5/n8 ), .CO(\intadd_5/n7 ), .S(\intadd_5/SUM[4] ) );
  FA_X1 \intadd_5/U7  ( .A(\intadd_5/A[5] ), .B(\intadd_5/B[5] ), .CI(
        \intadd_5/n7 ), .CO(\intadd_5/n6 ), .S(\intadd_5/SUM[5] ) );
  FA_X1 \intadd_5/U6  ( .A(\intadd_5/A[6] ), .B(\intadd_5/B[6] ), .CI(
        \intadd_5/n6 ), .CO(\intadd_5/n5 ), .S(\intadd_5/SUM[6] ) );
  FA_X1 \intadd_5/U5  ( .A(\intadd_5/A[7] ), .B(\intadd_5/B[7] ), .CI(
        \intadd_5/n5 ), .CO(\intadd_5/n4 ), .S(\intadd_5/SUM[7] ) );
  FA_X1 \intadd_5/U4  ( .A(\intadd_5/A[8] ), .B(\intadd_5/B[8] ), .CI(
        \intadd_5/n4 ), .CO(\intadd_5/n3 ), .S(\intadd_5/SUM[8] ) );
  FA_X1 \intadd_5/U3  ( .A(\intadd_5/A[9] ), .B(\intadd_5/B[9] ), .CI(
        \intadd_5/n3 ), .CO(\intadd_5/n2 ), .S(\intadd_5/SUM[9] ) );
  FA_X1 \intadd_5/U2  ( .A(\intadd_5/A[10] ), .B(\intadd_5/B[10] ), .CI(
        \intadd_5/n2 ), .CO(\intadd_5/n1 ), .S(\intadd_5/SUM[10] ) );
  FA_X1 \intadd_6/U10  ( .A(\intadd_6/A[0] ), .B(\intadd_6/B[0] ), .CI(
        \intadd_6/CI ), .CO(\intadd_6/n9 ), .S(\intadd_6/SUM[0] ) );
  FA_X1 \intadd_6/U9  ( .A(\intadd_6/A[1] ), .B(\intadd_6/B[1] ), .CI(
        \intadd_6/n9 ), .CO(\intadd_6/n8 ), .S(\intadd_6/SUM[1] ) );
  FA_X1 \intadd_6/U8  ( .A(\intadd_6/A[2] ), .B(\intadd_6/B[2] ), .CI(
        \intadd_6/n8 ), .CO(\intadd_6/n7 ), .S(\intadd_6/SUM[2] ) );
  FA_X1 \intadd_6/U7  ( .A(\intadd_6/A[3] ), .B(\intadd_6/B[3] ), .CI(
        \intadd_6/n7 ), .CO(\intadd_6/n6 ), .S(\intadd_6/SUM[3] ) );
  FA_X1 \intadd_6/U6  ( .A(\intadd_6/A[4] ), .B(\intadd_6/B[4] ), .CI(
        \intadd_6/n6 ), .CO(\intadd_6/n5 ), .S(\intadd_6/SUM[4] ) );
  FA_X1 \intadd_6/U5  ( .A(\intadd_6/A[5] ), .B(\intadd_6/B[5] ), .CI(
        \intadd_6/n5 ), .CO(\intadd_6/n4 ), .S(\intadd_6/SUM[5] ) );
  FA_X1 \intadd_6/U4  ( .A(\intadd_6/A[6] ), .B(\intadd_10/n1 ), .CI(
        \intadd_6/n4 ), .CO(\intadd_6/n3 ), .S(\intadd_6/SUM[6] ) );
  FA_X1 \intadd_6/U3  ( .A(\intadd_6/A[7] ), .B(\intadd_6/B[7] ), .CI(
        \intadd_6/n3 ), .CO(\intadd_6/n2 ), .S(\intadd_6/SUM[7] ) );
  FA_X1 \intadd_6/U2  ( .A(\intadd_6/A[8] ), .B(\intadd_6/B[8] ), .CI(
        \intadd_6/n2 ), .CO(\intadd_6/n1 ), .S(\intadd_6/SUM[8] ) );
  FA_X1 \intadd_7/U10  ( .A(\intadd_7/A[0] ), .B(\intadd_7/B[0] ), .CI(
        \intadd_7/CI ), .CO(\intadd_7/n9 ), .S(\intadd_5/A[3] ) );
  FA_X1 \intadd_7/U9  ( .A(\intadd_7/A[1] ), .B(\intadd_7/B[1] ), .CI(
        \intadd_7/n9 ), .CO(\intadd_7/n8 ), .S(\intadd_5/A[4] ) );
  FA_X1 \intadd_7/U8  ( .A(\intadd_7/A[2] ), .B(\intadd_7/B[2] ), .CI(
        \intadd_7/n8 ), .CO(\intadd_7/n7 ), .S(\intadd_5/A[5] ) );
  FA_X1 \intadd_7/U7  ( .A(\intadd_6/SUM[0] ), .B(\intadd_7/B[3] ), .CI(
        \intadd_7/n7 ), .CO(\intadd_7/n6 ), .S(\intadd_5/A[6] ) );
  FA_X1 \intadd_7/U6  ( .A(\intadd_6/SUM[1] ), .B(\intadd_7/B[4] ), .CI(
        \intadd_7/n6 ), .CO(\intadd_7/n5 ), .S(\intadd_5/A[7] ) );
  FA_X1 \intadd_7/U5  ( .A(\intadd_6/SUM[2] ), .B(\intadd_7/B[5] ), .CI(
        \intadd_7/n5 ), .CO(\intadd_7/n4 ), .S(\intadd_5/A[8] ) );
  FA_X1 \intadd_7/U4  ( .A(\intadd_6/SUM[3] ), .B(\intadd_7/B[6] ), .CI(
        \intadd_7/n4 ), .CO(\intadd_7/n3 ), .S(\intadd_5/A[9] ) );
  FA_X1 \intadd_7/U3  ( .A(\intadd_6/SUM[4] ), .B(\intadd_7/B[7] ), .CI(
        \intadd_7/n3 ), .CO(\intadd_7/n2 ), .S(\intadd_5/A[10] ) );
  FA_X1 \intadd_7/U2  ( .A(\intadd_6/SUM[5] ), .B(\intadd_7/B[8] ), .CI(
        \intadd_7/n2 ), .CO(\intadd_7/n1 ), .S(\intadd_7/SUM[8] ) );
  FA_X1 \intadd_8/U8  ( .A(\intadd_8/A[0] ), .B(\intadd_8/B[0] ), .CI(
        \intadd_8/CI ), .CO(\intadd_8/n7 ), .S(\intadd_8/SUM[0] ) );
  FA_X1 \intadd_8/U7  ( .A(\intadd_8/A[1] ), .B(\intadd_8/B[1] ), .CI(
        \intadd_8/n7 ), .CO(\intadd_8/n6 ), .S(\intadd_8/SUM[1] ) );
  FA_X1 \intadd_8/U6  ( .A(\intadd_8/A[2] ), .B(\intadd_8/B[2] ), .CI(
        \intadd_8/n6 ), .CO(\intadd_8/n5 ), .S(\intadd_8/SUM[2] ) );
  FA_X1 \intadd_8/U5  ( .A(\intadd_8/A[3] ), .B(\intadd_8/B[3] ), .CI(
        \intadd_8/n5 ), .CO(\intadd_8/n4 ), .S(\intadd_8/SUM[3] ) );
  FA_X1 \intadd_8/U4  ( .A(\intadd_8/A[4] ), .B(\intadd_8/B[4] ), .CI(
        \intadd_8/n4 ), .CO(\intadd_8/n3 ), .S(\intadd_8/SUM[4] ) );
  FA_X1 \intadd_8/U3  ( .A(\intadd_8/A[5] ), .B(\intadd_8/B[5] ), .CI(
        \intadd_8/n3 ), .CO(\intadd_8/n2 ), .S(\intadd_8/SUM[5] ) );
  FA_X1 \intadd_8/U2  ( .A(\intadd_8/A[6] ), .B(\intadd_8/B[6] ), .CI(
        \intadd_8/n2 ), .CO(\intadd_8/n1 ), .S(\intadd_8/SUM[6] ) );
  FA_X1 \intadd_9/U5  ( .A(\intadd_9/A[0] ), .B(\intadd_9/B[0] ), .CI(
        \intadd_9/CI ), .CO(\intadd_9/n4 ), .S(\intadd_9/SUM[0] ) );
  FA_X1 \intadd_9/U4  ( .A(\intadd_9/A[1] ), .B(\intadd_9/B[1] ), .CI(
        \intadd_9/n4 ), .CO(\intadd_9/n3 ), .S(\intadd_9/SUM[1] ) );
  FA_X1 \intadd_9/U3  ( .A(\intadd_9/A[2] ), .B(\intadd_9/B[2] ), .CI(
        \intadd_9/n3 ), .CO(\intadd_9/n2 ), .S(\intadd_9/SUM[2] ) );
  FA_X1 \intadd_9/U2  ( .A(\intadd_9/A[3] ), .B(\intadd_9/B[3] ), .CI(
        \intadd_9/n2 ), .CO(\intadd_9/n1 ), .S(\intadd_9/SUM[3] ) );
  FA_X1 \intadd_10/U4  ( .A(\intadd_10/A[0] ), .B(\intadd_10/B[0] ), .CI(
        \intadd_10/CI ), .CO(\intadd_10/n3 ), .S(\intadd_6/A[3] ) );
  FA_X1 \intadd_10/U3  ( .A(\intadd_10/A[1] ), .B(\intadd_10/B[1] ), .CI(
        \intadd_10/n3 ), .CO(\intadd_10/n2 ), .S(\intadd_6/A[4] ) );
  FA_X1 \intadd_10/U2  ( .A(\intadd_8/SUM[0] ), .B(\intadd_10/B[2] ), .CI(
        \intadd_10/n2 ), .CO(\intadd_10/n1 ), .S(\intadd_6/A[5] ) );
  FA_X1 \intadd_11/U4  ( .A(\intadd_11/A[0] ), .B(\intadd_11/B[0] ), .CI(
        \intadd_11/CI ), .CO(\intadd_11/n3 ), .S(\intadd_8/A[5] ) );
  FA_X1 \intadd_11/U3  ( .A(\intadd_9/SUM[0] ), .B(\intadd_11/B[1] ), .CI(
        \intadd_11/n3 ), .CO(\intadd_11/n2 ), .S(\intadd_8/A[6] ) );
  FA_X1 \intadd_11/U2  ( .A(\intadd_9/SUM[1] ), .B(\intadd_11/B[2] ), .CI(
        \intadd_11/n2 ), .CO(\intadd_11/n1 ), .S(\intadd_11/SUM[2] ) );
  FA_X1 \intadd_12/U4  ( .A(\intadd_8/SUM[2] ), .B(\intadd_12/B[0] ), .CI(
        \intadd_12/CI ), .CO(\intadd_12/n3 ), .S(\intadd_6/A[7] ) );
  FA_X1 \intadd_12/U3  ( .A(\intadd_8/SUM[3] ), .B(\intadd_12/B[1] ), .CI(
        \intadd_12/n3 ), .CO(\intadd_12/n2 ), .S(\intadd_6/A[8] ) );
  FA_X1 \intadd_12/U2  ( .A(\intadd_8/SUM[4] ), .B(\intadd_12/B[2] ), .CI(
        \intadd_12/n2 ), .CO(\intadd_12/n1 ), .S(\intadd_12/SUM[2] ) );
  FA_X1 \intadd_0/U28  ( .A(\intadd_0/A[0] ), .B(\intadd_0/B[0] ), .CI(
        \intadd_0/CI ), .CO(\intadd_0/n27 ), .S(\intadd_0/SUM[0] ) );
  FA_X1 \intadd_0/U27  ( .A(\intadd_0/A[1] ), .B(\intadd_0/B[1] ), .CI(
        \intadd_0/n27 ), .CO(\intadd_0/n26 ), .S(\intadd_0/SUM[1] ) );
  FA_X1 \intadd_0/U26  ( .A(\intadd_0/A[2] ), .B(\intadd_0/B[2] ), .CI(
        \intadd_0/n26 ), .CO(\intadd_0/n25 ), .S(\intadd_0/SUM[2] ) );
  FA_X1 \intadd_0/U25  ( .A(\intadd_5/SUM[0] ), .B(\intadd_0/B[3] ), .CI(
        \intadd_0/n25 ), .CO(\intadd_0/n24 ), .S(\intadd_0/SUM[3] ) );
  FA_X1 \intadd_0/U24  ( .A(\intadd_5/SUM[1] ), .B(\intadd_0/B[4] ), .CI(
        \intadd_0/n24 ), .CO(\intadd_0/n23 ), .S(\intadd_0/SUM[4] ) );
  FA_X1 \intadd_0/U23  ( .A(\intadd_5/SUM[2] ), .B(\intadd_0/B[5] ), .CI(
        \intadd_0/n23 ), .CO(\intadd_0/n22 ), .S(\intadd_0/SUM[5] ) );
  FA_X1 \intadd_0/U22  ( .A(\intadd_5/SUM[3] ), .B(\intadd_0/B[6] ), .CI(
        \intadd_0/n22 ), .CO(\intadd_0/n21 ), .S(\intadd_0/SUM[6] ) );
  FA_X1 \intadd_0/U21  ( .A(\intadd_5/SUM[4] ), .B(\intadd_0/B[7] ), .CI(
        \intadd_0/n21 ), .CO(\intadd_0/n20 ), .S(\intadd_0/SUM[7] ) );
  FA_X1 \intadd_0/U20  ( .A(\intadd_5/SUM[5] ), .B(\intadd_0/B[8] ), .CI(
        \intadd_0/n20 ), .CO(\intadd_0/n19 ), .S(\intadd_0/SUM[8] ) );
  FA_X1 \intadd_0/U19  ( .A(\intadd_5/SUM[6] ), .B(\intadd_0/B[9] ), .CI(
        \intadd_0/n19 ), .CO(\intadd_0/n18 ), .S(\intadd_0/SUM[9] ) );
  FA_X1 \intadd_0/U18  ( .A(\intadd_5/SUM[7] ), .B(\intadd_0/B[10] ), .CI(
        \intadd_0/n18 ), .CO(\intadd_0/n17 ), .S(\intadd_0/SUM[10] ) );
  FA_X1 \intadd_0/U17  ( .A(\intadd_5/SUM[8] ), .B(\intadd_0/B[11] ), .CI(
        \intadd_0/n17 ), .CO(\intadd_0/n16 ), .S(\intadd_0/SUM[11] ) );
  FA_X1 \intadd_0/U16  ( .A(\intadd_5/SUM[9] ), .B(\intadd_0/B[12] ), .CI(
        \intadd_0/n16 ), .CO(\intadd_0/n15 ), .S(\intadd_0/SUM[12] ) );
  FA_X1 \intadd_0/U15  ( .A(\intadd_5/SUM[10] ), .B(\intadd_0/B[13] ), .CI(
        \intadd_0/n15 ), .CO(\intadd_0/n14 ), .S(\intadd_0/SUM[13] ) );
  FA_X1 \intadd_0/U14  ( .A(\intadd_7/SUM[8] ), .B(\intadd_5/n1 ), .CI(
        \intadd_0/n14 ), .CO(\intadd_0/n13 ), .S(\intadd_0/SUM[14] ) );
  FA_X1 \intadd_0/U13  ( .A(\intadd_0/A[15] ), .B(\intadd_7/n1 ), .CI(
        \intadd_0/n13 ), .CO(\intadd_0/n12 ), .S(\intadd_0/SUM[15] ) );
  FA_X1 \intadd_0/U12  ( .A(\intadd_0/A[16] ), .B(\intadd_0/B[16] ), .CI(
        \intadd_0/n12 ), .CO(\intadd_0/n11 ), .S(\intadd_0/SUM[16] ) );
  FA_X1 \intadd_0/U11  ( .A(\intadd_6/SUM[8] ), .B(\intadd_0/B[17] ), .CI(
        \intadd_0/n11 ), .CO(\intadd_0/n10 ), .S(\intadd_0/SUM[17] ) );
  FA_X1 \intadd_0/U10  ( .A(\intadd_12/SUM[2] ), .B(\intadd_6/n1 ), .CI(
        \intadd_0/n10 ), .CO(\intadd_0/n9 ), .S(\intadd_0/SUM[18] ) );
  FA_X1 \intadd_0/U9  ( .A(\intadd_0/A[19] ), .B(\intadd_12/n1 ), .CI(
        \intadd_0/n9 ), .CO(\intadd_0/n8 ), .S(\intadd_0/SUM[19] ) );
  FA_X1 \intadd_0/U8  ( .A(\intadd_8/SUM[6] ), .B(\intadd_0/B[20] ), .CI(
        \intadd_0/n8 ), .CO(\intadd_0/n7 ), .S(\intadd_0/SUM[20] ) );
  FA_X1 \intadd_0/U7  ( .A(\intadd_11/SUM[2] ), .B(\intadd_8/n1 ), .CI(
        \intadd_0/n7 ), .CO(\intadd_0/n6 ), .S(\intadd_0/SUM[21] ) );
  FA_X1 \intadd_0/U6  ( .A(\intadd_0/A[22] ), .B(\intadd_11/n1 ), .CI(
        \intadd_0/n6 ), .CO(\intadd_0/n5 ), .S(\intadd_0/SUM[22] ) );
  FA_X1 \intadd_0/U5  ( .A(\intadd_9/SUM[3] ), .B(\intadd_0/B[23] ), .CI(
        \intadd_0/n5 ), .CO(\intadd_0/n4 ), .S(\intadd_0/SUM[23] ) );
  FA_X1 \intadd_0/U4  ( .A(\intadd_0/A[24] ), .B(\intadd_9/n1 ), .CI(
        \intadd_0/n4 ), .CO(\intadd_0/n3 ), .S(\intadd_0/SUM[24] ) );
  FA_X1 \intadd_0/U3  ( .A(\intadd_0/A[25] ), .B(\intadd_0/B[25] ), .CI(
        \intadd_0/n3 ), .CO(\intadd_0/n2 ), .S(\intadd_0/SUM[25] ) );
  FA_X1 \intadd_0/U2  ( .A(\intadd_0/A[26] ), .B(\intadd_0/B[26] ), .CI(
        \intadd_0/n2 ), .CO(\intadd_0/n1 ), .S(\intadd_0/SUM[26] ) );
  DFF_X1 \xbuf_real_p_imag_stage2_reg[16]  ( .D(xbuf_real_p_imag_stage1[16]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[16]), .QN(n2045) );
  DFF_X1 \twiddle_real_stage2_reg[16]  ( .D(twiddle_real[16]), .CK(clk), .Q(
        twiddle_real_stage2[16]), .QN(n2062) );
  DFF_X1 \twiddle_real_stage2_reg[15]  ( .D(twiddle_real[15]), .CK(clk), .Q(
        twiddle_real_stage2[15]), .QN(n2066) );
  DFF_X1 \twiddle_real_stage2_reg[14]  ( .D(twiddle_real[14]), .CK(clk), .Q(
        twiddle_real_stage2[14]), .QN(n2069) );
  DFF_X1 \twiddle_real_stage2_reg[13]  ( .D(twiddle_real[13]), .CK(clk), .Q(
        twiddle_real_stage2[13]) );
  DFF_X1 \twiddle_real_stage2_reg[12]  ( .D(twiddle_real[12]), .CK(clk), .Q(
        twiddle_real_stage2[12]) );
  DFF_X1 \twiddle_real_stage2_reg[11]  ( .D(twiddle_real[11]), .CK(clk), .Q(
        twiddle_real_stage2[11]) );
  DFF_X1 \twiddle_real_stage2_reg[10]  ( .D(twiddle_real[10]), .CK(clk), .Q(
        twiddle_real_stage2[10]) );
  DFF_X1 \twiddle_real_stage2_reg[9]  ( .D(twiddle_real[9]), .CK(clk), .Q(
        twiddle_real_stage2[9]) );
  DFF_X1 \twiddle_real_stage2_reg[8]  ( .D(twiddle_real[8]), .CK(clk), .Q(
        twiddle_real_stage2[8]) );
  DFF_X1 \twiddle_real_stage2_reg[7]  ( .D(twiddle_real[7]), .CK(clk), .Q(
        twiddle_real_stage2[7]) );
  DFF_X1 \twiddle_real_stage2_reg[6]  ( .D(twiddle_real[6]), .CK(clk), .Q(
        twiddle_real_stage2[6]) );
  DFF_X1 \twiddle_real_stage2_reg[5]  ( .D(twiddle_real[5]), .CK(clk), .Q(
        twiddle_real_stage2[5]) );
  DFF_X1 \twiddle_real_stage2_reg[4]  ( .D(twiddle_real[4]), .CK(clk), .Q(
        twiddle_real_stage2[4]) );
  DFF_X1 \twiddle_real_stage2_reg[3]  ( .D(twiddle_real[3]), .CK(clk), .Q(
        twiddle_real_stage2[3]) );
  DFF_X1 \twiddle_real_stage2_reg[1]  ( .D(twiddle_real[1]), .CK(clk), .Q(
        twiddle_real_stage2[1]), .QN(n2043) );
  DFF_X2 \xbuf_real_p_imag_stage2_reg[8]  ( .D(xbuf_real_p_imag_stage1[8]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[8]) );
  DFF_X2 \xbuf_real_p_imag_stage2_reg[11]  ( .D(xbuf_real_p_imag_stage1[11]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[11]), .QN(n2053) );
  DFF_X2 \xbuf_real_p_imag_stage2_reg[5]  ( .D(xbuf_real_p_imag_stage1[5]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[5]), .QN(n2049) );
  DFF_X1 \twiddle_real_stage2_reg[0]  ( .D(twiddle_real[0]), .CK(clk), .Q(
        twiddle_real_stage2[0]), .QN(n2050) );
  DFF_X2 \xbuf_imag_stage2_reg[1]  ( .D(xbuf_imag_stage1[1]), .CK(clk), .Q(
        xbuf_imag_stage2[1]), .QN(n38) );
  DFF_X2 \xbuf_real_stage2_reg[1]  ( .D(xbuf_real_stage1[1]), .CK(clk), .Q(
        xbuf_real_stage2[1]), .QN(n37) );
  DFF_X2 \twiddle_real_m_imag_stage2_reg[0]  ( .D(
        twiddle_real_p_imag_stage1[0]), .CK(clk), .Q(
        twiddle_real_m_imag_stage2[0]) );
  DFF_X2 \xbuf_real_stage2_reg[3]  ( .D(xbuf_real_stage1[3]), .CK(clk), .Q(
        xbuf_real_stage2[3]), .QN(n32) );
  DFF_X2 \xbuf_imag_stage2_reg[3]  ( .D(xbuf_imag_stage1[3]), .CK(clk), .Q(
        xbuf_imag_stage2[3]), .QN(n21) );
  DFF_X2 \xbuf_imag_stage2_reg[9]  ( .D(xbuf_imag_stage1[9]), .CK(clk), .Q(
        xbuf_imag_stage2[9]), .QN(n18) );
  DFF_X2 \xbuf_real_stage2_reg[9]  ( .D(xbuf_real_stage1[9]), .CK(clk), .Q(
        xbuf_real_stage2[9]), .QN(n12) );
  DFF_X2 \xbuf_imag_stage2_reg[13]  ( .D(xbuf_imag_stage1[13]), .CK(clk), .Q(
        xbuf_imag_stage2[13]), .QN(n16) );
  DFF_X2 \xbuf_real_stage2_reg[11]  ( .D(xbuf_real_stage1[11]), .CK(clk), .Q(
        xbuf_real_stage2[11]), .QN(n11) );
  DFF_X2 \xbuf_real_stage2_reg[13]  ( .D(xbuf_real_stage1[13]), .CK(clk), .Q(
        xbuf_real_stage2[13]), .QN(n10) );
  DFF_X2 \xbuf_imag_stage2_reg[15]  ( .D(xbuf_imag_stage1[15]), .CK(clk), .Q(
        xbuf_imag_stage2[15]), .QN(n15) );
  DFF_X2 \xbuf_real_stage2_reg[15]  ( .D(xbuf_real_stage1[15]), .CK(clk), .Q(
        xbuf_real_stage2[15]), .QN(n9) );
  DFF_X2 \xbuf_imag_stage2_reg[7]  ( .D(xbuf_imag_stage1[7]), .CK(clk), .Q(
        xbuf_imag_stage2[7]), .QN(n19) );
  DFF_X2 \xbuf_imag_stage2_reg[5]  ( .D(xbuf_imag_stage1[5]), .CK(clk), .Q(
        xbuf_imag_stage2[5]), .QN(n20) );
  DFF_X2 \xbuf_imag_stage2_reg[11]  ( .D(xbuf_imag_stage1[11]), .CK(clk), .Q(
        xbuf_imag_stage2[11]), .QN(n17) );
  DFF_X2 \xbuf_real_stage2_reg[5]  ( .D(xbuf_real_stage1[5]), .CK(clk), .Q(
        xbuf_real_stage2[5]), .QN(n14) );
  DFF_X2 \xbuf_real_stage2_reg[7]  ( .D(xbuf_real_stage1[7]), .CK(clk), .Q(
        xbuf_real_stage2[7]), .QN(n13) );
  DFF_X2 \xbuf_real_p_imag_stage2_reg[14]  ( .D(xbuf_real_p_imag_stage1[14]), 
        .CK(clk), .Q(xbuf_real_p_imag_stage2[14]), .QN(n1) );
  INV_X1 U3 ( .A(n1308), .ZN(n1650) );
  INV_X1 U4 ( .A(n1671), .ZN(n1709) );
  INV_X2 U5 ( .A(xbuf_real_p_imag_stage2[2]), .ZN(n1784) );
  OAI21_X1 U6 ( .B1(twiddle_real_stage2[2]), .B2(twiddle_real_stage2[0]), .A(
        twiddle_real_stage2[1]), .ZN(n1334) );
  INV_X1 U7 ( .A(n1674), .ZN(n1710) );
  INV_X1 U8 ( .A(n1732), .ZN(n1778) );
  OR2_X1 U9 ( .A1(\yr[31] ), .A2(\intadd_1/SUM[14] ), .ZN(n2040) );
  NOR2_X1 U10 ( .A1(n1718), .A2(n1719), .ZN(n1717) );
  INV_X1 U11 ( .A(n1782), .ZN(n1774) );
  OR2_X1 U12 ( .A1(n33), .A2(n147), .ZN(n529) );
  OR2_X1 U13 ( .A1(n22), .A2(n674), .ZN(n1055) );
  OAI22_X1 U14 ( .A1(n533), .A2(n445), .B1(n429), .B2(n3), .ZN(n442) );
  AND2_X1 U15 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n412), .ZN(n431) );
  NAND2_X1 U16 ( .A1(n630), .A2(n1044), .ZN(n1035) );
  XNOR2_X1 U17 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n413) );
  XOR2_X1 U18 ( .A(xbuf_real_stage2[9]), .B(xbuf_real_stage2[8]), .Z(n56) );
  OAI22_X1 U19 ( .A1(n1061), .A2(n973), .B1(n954), .B2(n2), .ZN(n967) );
  XOR2_X1 U20 ( .A(xbuf_imag_stage2[9]), .B(xbuf_imag_stage2[8]), .Z(n583) );
  NAND2_X1 U21 ( .A1(n101), .A2(n516), .ZN(n509) );
  OAI22_X1 U22 ( .A1(n533), .A2(n485), .B1(n464), .B2(n3), .ZN(n489) );
  AND2_X1 U23 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n447), .ZN(n472) );
  XOR2_X1 U24 ( .A(xbuf_real_stage2[11]), .B(xbuf_real_stage2[10]), .Z(n42) );
  OAI22_X1 U25 ( .A1(n1061), .A2(n1013), .B1(n991), .B2(n2), .ZN(n1015) );
  XOR2_X1 U26 ( .A(xbuf_imag_stage2[13]), .B(xbuf_imag_stage2[12]), .Z(n570)
         );
  NAND2_X1 U27 ( .A1(xbuf_real_stage2[1]), .A2(n3), .ZN(n533) );
  OR2_X1 U28 ( .A1(n555), .A2(n554), .ZN(n552) );
  NAND2_X1 U29 ( .A1(xbuf_imag_stage2[1]), .A2(n2), .ZN(n1061) );
  OR2_X1 U30 ( .A1(n1082), .A2(n1081), .ZN(n1079) );
  NAND2_X1 U31 ( .A1(n571), .A2(n1088), .ZN(n1090) );
  NAND2_X1 U32 ( .A1(n44), .A2(n561), .ZN(n563) );
  OAI22_X1 U33 ( .A1(n563), .A2(n9), .B1(n264), .B2(n561), .ZN(n293) );
  NAND2_X1 U34 ( .A1(n43), .A2(n548), .ZN(n547) );
  OAI22_X1 U35 ( .A1(n547), .A2(n10), .B1(n319), .B2(n548), .ZN(n369) );
  OAI22_X1 U36 ( .A1(n547), .A2(n316), .B1(n548), .B2(n315), .ZN(n366) );
  OAI22_X1 U37 ( .A1(n547), .A2(n315), .B1(n548), .B2(n296), .ZN(n346) );
  NOR2_X2 U38 ( .A1(n1315), .A2(n1317), .ZN(n1548) );
  XOR2_X1 U39 ( .A(xbuf_real_stage2[5]), .B(xbuf_real_stage2[4]), .Z(n101) );
  XOR2_X1 U40 ( .A(xbuf_imag_stage2[11]), .B(xbuf_imag_stage2[10]), .Z(n569)
         );
  NOR2_X2 U41 ( .A1(n1466), .A2(n1465), .ZN(n1586) );
  NOR2_X2 U42 ( .A1(n1792), .A2(n1318), .ZN(n1547) );
  NOR2_X2 U43 ( .A1(n1449), .A2(n1304), .ZN(n1648) );
  NOR2_X2 U44 ( .A1(n1606), .A2(n1605), .ZN(n1707) );
  NOR2_X2 U45 ( .A1(n1319), .A2(n1793), .ZN(n1550) );
  XOR2_X1 U46 ( .A(xbuf_imag_stage2[5]), .B(xbuf_imag_stage2[4]), .Z(n630) );
  XNOR2_X1 U47 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n1012) );
  XOR2_X1 U48 ( .A(xbuf_imag_stage2[7]), .B(xbuf_imag_stage2[6]), .Z(n598) );
  XNOR2_X1 U49 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n972) );
  XNOR2_X1 U50 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n953) );
  XOR2_X1 U51 ( .A(xbuf_real_stage2[15]), .B(xbuf_real_stage2[14]), .Z(n44) );
  XOR2_X1 U52 ( .A(xbuf_imag_stage2[15]), .B(xbuf_imag_stage2[14]), .Z(n571)
         );
  OAI22_X1 U53 ( .A1(n529), .A2(n374), .B1(n353), .B2(n531), .ZN(n385) );
  OAI22_X1 U54 ( .A1(n529), .A2(n423), .B1(n407), .B2(n531), .ZN(n451) );
  OAI22_X1 U55 ( .A1(n529), .A2(n462), .B1(n423), .B2(n531), .ZN(n460) );
  OAI22_X1 U56 ( .A1(n529), .A2(n474), .B1(n462), .B2(n531), .ZN(n480) );
  OAI22_X1 U57 ( .A1(n529), .A2(n488), .B1(n474), .B2(n531), .ZN(n493) );
  OAI22_X1 U58 ( .A1(n529), .A2(n497), .B1(n488), .B2(n531), .ZN(n500) );
  XOR2_X1 U59 ( .A(xbuf_real_stage2[13]), .B(xbuf_real_stage2[12]), .Z(n43) );
  NAND2_X1 U60 ( .A1(n570), .A2(n1075), .ZN(n1074) );
  OAI22_X1 U61 ( .A1(n874), .A2(n1074), .B1(n1075), .B2(n873), .ZN(n898) );
  OAI22_X1 U62 ( .A1(n871), .A2(n1075), .B1(n1074), .B2(n16), .ZN(n900) );
  OAI22_X1 U63 ( .A1(n1074), .A2(n873), .B1(n1075), .B2(n813), .ZN(n869) );
  OAI22_X1 U64 ( .A1(n1055), .A2(n894), .B1(n882), .B2(n1057), .ZN(n913) );
  OAI22_X1 U65 ( .A1(n1055), .A2(n974), .B1(n955), .B2(n1057), .ZN(n986) );
  OAI22_X1 U66 ( .A1(n1055), .A2(n955), .B1(n940), .B2(n1057), .ZN(n950) );
  OAI22_X1 U67 ( .A1(n1055), .A2(n999), .B1(n974), .B2(n1057), .ZN(n1003) );
  OAI22_X1 U68 ( .A1(n1055), .A2(n1014), .B1(n999), .B2(n1057), .ZN(n1018) );
  OAI22_X1 U69 ( .A1(n1055), .A2(n1023), .B1(n1014), .B2(n1057), .ZN(n1027) );
  OAI22_X1 U70 ( .A1(n1055), .A2(n1042), .B1(n1023), .B2(n1057), .ZN(n1038) );
  OAI22_X1 U71 ( .A1(n428), .A2(n350), .B1(n446), .B2(n301), .ZN(n352) );
  OAI22_X1 U72 ( .A1(n428), .A2(n371), .B1(n446), .B2(n350), .ZN(n386) );
  OAI22_X1 U73 ( .A1(n428), .A2(n413), .B1(n446), .B2(n371), .ZN(n394) );
  OAI22_X1 U74 ( .A1(n428), .A2(n424), .B1(n446), .B2(n413), .ZN(n430) );
  OAI22_X1 U75 ( .A1(n428), .A2(n12), .B1(n427), .B2(n446), .ZN(n443) );
  NAND2_X1 U76 ( .A1(n56), .A2(n446), .ZN(n428) );
  XNOR2_X2 U77 ( .A(xbuf_real_stage2[9]), .B(xbuf_real_stage2[10]), .ZN(n4) );
  NAND2_X1 U78 ( .A1(n42), .A2(n411), .ZN(n392) );
  OAI22_X1 U79 ( .A1(n392), .A2(n11), .B1(n375), .B2(n4), .ZN(n415) );
  OAI22_X1 U80 ( .A1(n392), .A2(n390), .B1(n354), .B2(n4), .ZN(n384) );
  OAI22_X1 U81 ( .A1(n392), .A2(n391), .B1(n390), .B2(n4), .ZN(n434) );
  OAI22_X1 U82 ( .A1(n392), .A2(n354), .B1(n318), .B2(n4), .ZN(n370) );
  XNOR2_X1 U83 ( .A(xbuf_real_stage2[9]), .B(xbuf_real_stage2[10]), .ZN(n411)
         );
  XNOR2_X2 U84 ( .A(xbuf_imag_stage2[9]), .B(xbuf_imag_stage2[10]), .ZN(n5) );
  OAI22_X1 U85 ( .A1(n918), .A2(n851), .B1(n825), .B2(n5), .ZN(n855) );
  OAI22_X1 U86 ( .A1(n918), .A2(n875), .B1(n851), .B2(n5), .ZN(n896) );
  OAI22_X1 U87 ( .A1(n919), .A2(n5), .B1(n918), .B2(n17), .ZN(n959) );
  OAI22_X1 U88 ( .A1(n917), .A2(n918), .B1(n916), .B2(n5), .ZN(n960) );
  OAI22_X1 U89 ( .A1(n918), .A2(n916), .B1(n875), .B2(n5), .ZN(n912) );
  NAND2_X1 U90 ( .A1(n569), .A2(n936), .ZN(n918) );
  XNOR2_X1 U91 ( .A(xbuf_imag_stage2[9]), .B(xbuf_imag_stage2[10]), .ZN(n936)
         );
  OAI22_X1 U92 ( .A1(n970), .A2(n826), .B1(n987), .B2(n789), .ZN(n822) );
  OAI22_X1 U93 ( .A1(n970), .A2(n876), .B1(n987), .B2(n827), .ZN(n879) );
  OAI22_X1 U94 ( .A1(n970), .A2(n889), .B1(n987), .B2(n876), .ZN(n911) );
  OAI22_X1 U95 ( .A1(n970), .A2(n935), .B1(n987), .B2(n889), .ZN(n921) );
  OAI22_X1 U96 ( .A1(n970), .A2(n969), .B1(n987), .B2(n935), .ZN(n976) );
  OAI22_X1 U97 ( .A1(n957), .A2(n987), .B1(n970), .B2(n18), .ZN(n984) );
  OAI22_X1 U98 ( .A1(n971), .A2(n970), .B1(n987), .B2(n969), .ZN(n993) );
  NAND2_X1 U99 ( .A1(n583), .A2(n987), .ZN(n970) );
  XNOR2_X2 U100 ( .A(xbuf_imag_stage2[3]), .B(xbuf_imag_stage2[4]), .ZN(n6) );
  OAI22_X1 U101 ( .A1(n1035), .A2(n877), .B1(n6), .B2(n850), .ZN(n897) );
  OAI22_X1 U102 ( .A1(n1035), .A2(n892), .B1(n6), .B2(n877), .ZN(n910) );
  OAI22_X1 U103 ( .A1(n1035), .A2(n956), .B1(n6), .B2(n934), .ZN(n977) );
  OAI22_X1 U104 ( .A1(n1035), .A2(n934), .B1(n6), .B2(n892), .ZN(n943) );
  OAI22_X1 U105 ( .A1(n1035), .A2(n989), .B1(n6), .B2(n956), .ZN(n985) );
  OAI22_X1 U106 ( .A1(n1035), .A2(n990), .B1(n6), .B2(n989), .ZN(n1007) );
  OAI22_X1 U107 ( .A1(n1035), .A2(n1012), .B1(n6), .B2(n990), .ZN(n1016) );
  OAI22_X1 U108 ( .A1(n1035), .A2(n1033), .B1(n6), .B2(n1012), .ZN(n1029) );
  XNOR2_X1 U109 ( .A(xbuf_imag_stage2[3]), .B(xbuf_imag_stage2[4]), .ZN(n1044)
         );
  OAI22_X1 U110 ( .A1(n477), .A2(n389), .B1(n486), .B2(n373), .ZN(n405) );
  OAI22_X1 U111 ( .A1(n477), .A2(n373), .B1(n486), .B2(n320), .ZN(n368) );
  OAI22_X1 U112 ( .A1(n477), .A2(n408), .B1(n486), .B2(n389), .ZN(n435) );
  OAI22_X1 U113 ( .A1(n477), .A2(n444), .B1(n486), .B2(n408), .ZN(n450) );
  OAI22_X1 U114 ( .A1(n477), .A2(n448), .B1(n486), .B2(n444), .ZN(n466) );
  OAI22_X1 U115 ( .A1(n477), .A2(n475), .B1(n486), .B2(n448), .ZN(n471) );
  OAI22_X1 U116 ( .A1(n477), .A2(n13), .B1(n463), .B2(n486), .ZN(n490) );
  OAI22_X1 U117 ( .A1(n477), .A2(n476), .B1(n486), .B2(n475), .ZN(n492) );
  NAND2_X1 U118 ( .A1(n71), .A2(n486), .ZN(n477) );
  OAI22_X1 U119 ( .A1(n1001), .A2(n872), .B1(n1025), .B2(n814), .ZN(n849) );
  OAI22_X1 U120 ( .A1(n1001), .A2(n880), .B1(n1025), .B2(n872), .ZN(n899) );
  OAI22_X1 U121 ( .A1(n1001), .A2(n893), .B1(n1025), .B2(n880), .ZN(n915) );
  OAI22_X1 U122 ( .A1(n1001), .A2(n938), .B1(n1025), .B2(n893), .ZN(n942) );
  OAI22_X1 U123 ( .A1(n1001), .A2(n953), .B1(n1025), .B2(n938), .ZN(n952) );
  OAI22_X1 U124 ( .A1(n1001), .A2(n972), .B1(n1025), .B2(n953), .ZN(n968) );
  OAI22_X1 U125 ( .A1(n1001), .A2(n1000), .B1(n1025), .B2(n972), .ZN(n1005) );
  OAI22_X1 U126 ( .A1(n998), .A2(n1025), .B1(n1001), .B2(n19), .ZN(n1019) );
  NAND2_X1 U127 ( .A1(n598), .A2(n1025), .ZN(n1001) );
  XNOR2_X2 U128 ( .A(xbuf_real_stage2[3]), .B(xbuf_real_stage2[4]), .ZN(n7) );
  OAI22_X1 U129 ( .A1(n509), .A2(n355), .B1(n317), .B2(n7), .ZN(n365) );
  OAI22_X1 U130 ( .A1(n509), .A2(n376), .B1(n355), .B2(n7), .ZN(n383) );
  OAI22_X1 U131 ( .A1(n509), .A2(n426), .B1(n409), .B2(n7), .ZN(n449) );
  OAI22_X1 U132 ( .A1(n509), .A2(n409), .B1(n376), .B2(n7), .ZN(n414) );
  OAI22_X1 U133 ( .A1(n509), .A2(n461), .B1(n426), .B2(n7), .ZN(n458) );
  OAI22_X1 U134 ( .A1(n509), .A2(n478), .B1(n461), .B2(n7), .ZN(n481) );
  OAI22_X1 U135 ( .A1(n509), .A2(n507), .B1(n499), .B2(n7), .ZN(n504) );
  OAI22_X1 U136 ( .A1(n509), .A2(n499), .B1(n478), .B2(n7), .ZN(n491) );
  OAI22_X1 U137 ( .A1(n509), .A2(n14), .B1(n506), .B2(n7), .ZN(n514) );
  OAI22_X1 U138 ( .A1(n509), .A2(n508), .B1(n507), .B2(n7), .ZN(n513) );
  XNOR2_X1 U139 ( .A(xbuf_real_stage2[3]), .B(xbuf_real_stage2[4]), .ZN(n516)
         );
  NOR2_X2 U140 ( .A1(n1306), .A2(n1450), .ZN(n1651) );
  NOR2_X2 U141 ( .A1(n1302), .A2(n1305), .ZN(n1649) );
  NOR2_X2 U142 ( .A1(n1318), .A2(n1319), .ZN(n1549) );
  NOR2_X2 U143 ( .A1(n1465), .A2(n1464), .ZN(n1588) );
  NOR2_X2 U144 ( .A1(n1669), .A2(n1604), .ZN(n1708) );
  NOR3_X4 U145 ( .A1(xbuf_real_p_imag_stage2[0]), .A2(
        xbuf_real_p_imag_stage2[1]), .A3(n1784), .ZN(n1779) );
  AOI221_X4 U146 ( .B1(xbuf_real_p_imag_stage2[15]), .B2(
        xbuf_real_p_imag_stage2[16]), .C1(n2057), .C2(n2045), .A(n1798), .ZN(
        n1797) );
  AOI22_X2 U147 ( .A1(xbuf_real_p_imag_stage2[14]), .A2(
        xbuf_real_p_imag_stage2[15]), .B1(n2057), .B2(n1), .ZN(n1798) );
  AND2_X1 U148 ( .A1(n35), .A2(n1269), .ZN(n8) );
  XNOR2_X1 U149 ( .A(xbuf_imag_stage2[3]), .B(xbuf_imag_stage2[2]), .ZN(n22)
         );
  AND2_X1 U150 ( .A1(n1078), .A2(n1077), .ZN(n23) );
  AND2_X1 U151 ( .A1(n633), .A2(n632), .ZN(n24) );
  AND2_X1 U152 ( .A1(n600), .A2(n599), .ZN(n25) );
  AND2_X1 U153 ( .A1(n585), .A2(n584), .ZN(n26) );
  AND2_X1 U154 ( .A1(n574), .A2(n573), .ZN(n27) );
  AND2_X1 U155 ( .A1(n551), .A2(n550), .ZN(n28) );
  AND2_X1 U156 ( .A1(n73), .A2(n72), .ZN(n29) );
  AND2_X1 U157 ( .A1(n58), .A2(n57), .ZN(n30) );
  AND2_X1 U158 ( .A1(n47), .A2(n46), .ZN(n31) );
  XNOR2_X1 U159 ( .A(xbuf_real_stage2[3]), .B(xbuf_real_stage2[2]), .ZN(n33)
         );
  AND2_X1 U160 ( .A1(n104), .A2(n103), .ZN(n34) );
  OR2_X1 U161 ( .A1(n1268), .A2(n1267), .ZN(n35) );
  AND2_X1 U162 ( .A1(n149), .A2(n148), .ZN(n36) );
  INV_X1 U163 ( .A(n531), .ZN(n147) );
  OR2_X1 U164 ( .A1(n1181), .A2(n1180), .ZN(n39) );
  AND2_X1 U165 ( .A1(n39), .A2(n1182), .ZN(n40) );
  AND2_X1 U166 ( .A1(n676), .A2(n675), .ZN(n41) );
  INV_X1 U167 ( .A(n1057), .ZN(n674) );
  INV_X1 U168 ( .A(n206), .ZN(n184) );
  OR2_X1 U169 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n9), .ZN(n264) );
  XNOR2_X1 U170 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n235) );
  OR2_X1 U171 ( .A1(n680), .A2(n1055), .ZN(n676) );
  XOR2_X1 U172 ( .A(xbuf_real_stage2[7]), .B(xbuf_real_stage2[6]), .Z(n71) );
  XNOR2_X1 U173 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n354) );
  XNOR2_X1 U174 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n300) );
  XNOR2_X1 U175 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n243) );
  XNOR2_X1 U176 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n159) );
  XNOR2_X1 U177 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n131) );
  XNOR2_X1 U178 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n98) );
  XNOR2_X1 U179 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n940) );
  XNOR2_X1 U180 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n850) );
  XNOR2_X1 U181 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n793) );
  XNOR2_X1 U182 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[15]), 
        .ZN(n824) );
  XNOR2_X1 U183 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n705) );
  OAI22_X1 U184 ( .A1(n970), .A2(n736), .B1(n987), .B2(n702), .ZN(n732) );
  XNOR2_X1 U185 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n636) );
  OR2_X1 U186 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n37), .ZN(n534) );
  XNOR2_X1 U187 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n485) );
  XNOR2_X1 U188 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n426) );
  OAI22_X1 U189 ( .A1(n529), .A2(n407), .B1(n374), .B2(n531), .ZN(n416) );
  OAI22_X1 U190 ( .A1(n428), .A2(n301), .B1(n446), .B2(n300), .ZN(n321) );
  OAI22_X1 U191 ( .A1(n392), .A2(n318), .B1(n287), .B2(n4), .ZN(n343) );
  OAI22_X1 U192 ( .A1(n529), .A2(n214), .B1(n180), .B2(n531), .ZN(n217) );
  OAI22_X1 U193 ( .A1(n428), .A2(n175), .B1(n446), .B2(n160), .ZN(n182) );
  OAI22_X1 U194 ( .A1(n477), .A2(n134), .B1(n486), .B2(n113), .ZN(n137) );
  OAI22_X1 U195 ( .A1(n428), .A2(n98), .B1(n446), .B2(n91), .ZN(n96) );
  XNOR2_X1 U196 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n62) );
  XNOR2_X1 U197 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n53) );
  XNOR2_X1 U198 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n1000) );
  INV_X1 U199 ( .A(n5), .ZN(n937) );
  OAI22_X1 U200 ( .A1(n1055), .A2(n940), .B1(n894), .B2(n1057), .ZN(n941) );
  OAI22_X1 U201 ( .A1(n970), .A2(n827), .B1(n987), .B2(n826), .ZN(n854) );
  AND2_X1 U202 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n812), .ZN(n870) );
  OAI22_X1 U203 ( .A1(n918), .A2(n741), .B1(n706), .B2(n5), .ZN(n745) );
  OAI22_X1 U204 ( .A1(n918), .A2(n706), .B1(n686), .B2(n5), .ZN(n718) );
  INV_X1 U205 ( .A(n654), .ZN(n681) );
  OAI22_X1 U206 ( .A1(n918), .A2(n629), .B1(n617), .B2(n5), .ZN(n627) );
  XNOR2_X1 U207 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n589) );
  XNOR2_X1 U208 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n575) );
  OR2_X1 U209 ( .A1(n522), .A2(n531), .ZN(n523) );
  OAI22_X1 U210 ( .A1(n529), .A2(n518), .B1(n497), .B2(n531), .ZN(n511) );
  OAI22_X1 U211 ( .A1(n428), .A2(n425), .B1(n446), .B2(n424), .ZN(n459) );
  OAI22_X1 U212 ( .A1(n392), .A2(n68), .B1(n62), .B2(n4), .ZN(n85) );
  OAI22_X1 U213 ( .A1(n547), .A2(n59), .B1(n548), .B2(n49), .ZN(n51) );
  XNOR2_X1 U214 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n1033) );
  OAI22_X1 U215 ( .A1(n1002), .A2(n1001), .B1(n1025), .B2(n1000), .ZN(n1017)
         );
  AND2_X1 U216 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n891), .ZN(n932) );
  OAI22_X1 U217 ( .A1(n1074), .A2(n639), .B1(n1075), .B2(n638), .ZN(n666) );
  OAI22_X1 U218 ( .A1(n1074), .A2(n576), .B1(n1075), .B2(n1065), .ZN(n1066) );
  XNOR2_X1 U219 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n530) );
  XNOR2_X1 U220 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n1054) );
  OAI22_X1 U221 ( .A1(n1074), .A2(n1065), .B1(n1075), .B2(n1076), .ZN(n1071)
         );
  XNOR2_X1 U222 ( .A(n554), .B(n555), .ZN(n542) );
  XNOR2_X1 U223 ( .A(n1081), .B(n1082), .ZN(n1069) );
  XOR2_X1 U224 ( .A(n1093), .B(n1092), .Z(n1094) );
  AND2_X1 U225 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(xbuf_real_stage2[0]), 
        .ZN(tmp_i[0]) );
  XNOR2_X1 U226 ( .A(n553), .B(n542), .ZN(tmp_i[29]) );
  XNOR2_X1 U227 ( .A(n1080), .B(n1069), .ZN(tmp_r[29]) );
  XNOR2_X1 U228 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n55) );
  XNOR2_X1 U229 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n45) );
  OAI22_X1 U230 ( .A1(n392), .A2(n55), .B1(n45), .B2(n4), .ZN(n52) );
  XNOR2_X2 U231 ( .A(xbuf_real_stage2[11]), .B(xbuf_real_stage2[12]), .ZN(n548) );
  XNOR2_X1 U232 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n59) );
  XNOR2_X1 U233 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n49) );
  XNOR2_X2 U234 ( .A(xbuf_real_stage2[13]), .B(xbuf_real_stage2[14]), .ZN(n561) );
  XNOR2_X1 U235 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n48) );
  OAI22_X1 U236 ( .A1(n563), .A2(n53), .B1(n561), .B2(n48), .ZN(n540) );
  INV_X1 U237 ( .A(n540), .ZN(n50) );
  OR2_X1 U238 ( .A1(n45), .A2(n392), .ZN(n47) );
  OR2_X1 U239 ( .A1(n45), .A2(n4), .ZN(n46) );
  XNOR2_X1 U240 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n537) );
  OAI22_X1 U241 ( .A1(n563), .A2(n48), .B1(n561), .B2(n537), .ZN(n541) );
  XNOR2_X1 U242 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n538) );
  OAI22_X1 U243 ( .A1(n547), .A2(n49), .B1(n548), .B2(n538), .ZN(n539) );
  FA_X1 U244 ( .A(n52), .B(n51), .CI(n50), .CO(n536), .S(n65) );
  XNOR2_X1 U245 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n54) );
  OAI22_X1 U246 ( .A1(n563), .A2(n54), .B1(n561), .B2(n53), .ZN(n76) );
  XNOR2_X1 U247 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n66) );
  OAI22_X1 U248 ( .A1(n563), .A2(n66), .B1(n561), .B2(n54), .ZN(n75) );
  OAI22_X1 U249 ( .A1(n392), .A2(n62), .B1(n55), .B2(n4), .ZN(n74) );
  XNOR2_X1 U250 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n61) );
  XNOR2_X2 U251 ( .A(xbuf_real_stage2[7]), .B(xbuf_real_stage2[8]), .ZN(n446)
         );
  OR2_X1 U252 ( .A1(n61), .A2(n428), .ZN(n58) );
  OR2_X1 U253 ( .A1(n61), .A2(n446), .ZN(n57) );
  XNOR2_X1 U254 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n60) );
  OAI22_X1 U255 ( .A1(n547), .A2(n60), .B1(n548), .B2(n59), .ZN(n78) );
  XNOR2_X1 U256 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n69) );
  OAI22_X1 U257 ( .A1(n547), .A2(n69), .B1(n548), .B2(n60), .ZN(n87) );
  XNOR2_X1 U258 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n70) );
  OAI22_X1 U259 ( .A1(n428), .A2(n70), .B1(n446), .B2(n61), .ZN(n86) );
  XNOR2_X1 U260 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n68) );
  FA_X1 U261 ( .A(n65), .B(n64), .CI(n63), .CO(n1187), .S(n1191) );
  XNOR2_X1 U262 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n67) );
  OAI22_X1 U263 ( .A1(n563), .A2(n67), .B1(n561), .B2(n66), .ZN(n89) );
  XNOR2_X1 U264 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n110) );
  OAI22_X1 U265 ( .A1(n563), .A2(n110), .B1(n561), .B2(n67), .ZN(n107) );
  XNOR2_X1 U266 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n90) );
  OAI22_X1 U267 ( .A1(n392), .A2(n90), .B1(n68), .B2(n4), .ZN(n88) );
  INV_X1 U268 ( .A(n75), .ZN(n83) );
  XNOR2_X1 U269 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n108) );
  OAI22_X1 U270 ( .A1(n547), .A2(n108), .B1(n548), .B2(n69), .ZN(n94) );
  XNOR2_X1 U271 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n91) );
  OAI22_X1 U272 ( .A1(n428), .A2(n91), .B1(n446), .B2(n70), .ZN(n93) );
  XNOR2_X1 U273 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n92) );
  XNOR2_X2 U274 ( .A(xbuf_real_stage2[5]), .B(xbuf_real_stage2[6]), .ZN(n486)
         );
  OR2_X1 U275 ( .A1(n92), .A2(n477), .ZN(n73) );
  OR2_X1 U276 ( .A1(n92), .A2(n486), .ZN(n72) );
  FA_X1 U277 ( .A(n76), .B(n75), .CI(n74), .CO(n64), .S(n80) );
  FA_X1 U278 ( .A(n30), .B(n78), .CI(n77), .CO(n63), .S(n79) );
  FA_X1 U279 ( .A(n81), .B(n80), .CI(n79), .CO(n1190), .S(n1194) );
  FA_X1 U280 ( .A(n84), .B(n83), .CI(n82), .CO(n81), .S(n119) );
  FA_X1 U281 ( .A(n87), .B(n86), .CI(n85), .CO(n77), .S(n118) );
  FA_X1 U282 ( .A(n89), .B(n107), .CI(n88), .CO(n84), .S(n116) );
  XNOR2_X1 U283 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n99) );
  OAI22_X1 U284 ( .A1(n392), .A2(n99), .B1(n90), .B2(n4), .ZN(n97) );
  XNOR2_X1 U285 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n112) );
  OAI22_X1 U286 ( .A1(n477), .A2(n112), .B1(n486), .B2(n92), .ZN(n95) );
  FA_X1 U287 ( .A(n94), .B(n93), .CI(n29), .CO(n82), .S(n114) );
  FA_X1 U288 ( .A(n97), .B(n96), .CI(n95), .CO(n115), .S(n143) );
  XNOR2_X1 U289 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n106) );
  XNOR2_X1 U290 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n109) );
  OAI22_X1 U291 ( .A1(n547), .A2(n106), .B1(n548), .B2(n109), .ZN(n125) );
  XNOR2_X1 U292 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n100) );
  OAI22_X1 U293 ( .A1(n428), .A2(n100), .B1(n446), .B2(n98), .ZN(n124) );
  XNOR2_X1 U294 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n105) );
  OAI22_X1 U295 ( .A1(n392), .A2(n105), .B1(n99), .B2(n4), .ZN(n123) );
  XNOR2_X1 U296 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n134) );
  XNOR2_X1 U297 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n113) );
  XNOR2_X1 U298 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n130) );
  OAI22_X1 U299 ( .A1(n428), .A2(n130), .B1(n446), .B2(n100), .ZN(n136) );
  XNOR2_X1 U300 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n102) );
  OAI22_X1 U301 ( .A1(n509), .A2(n131), .B1(n102), .B2(n7), .ZN(n135) );
  OR2_X1 U302 ( .A1(n102), .A2(n509), .ZN(n104) );
  OR2_X1 U303 ( .A1(n102), .A2(n7), .ZN(n103) );
  XNOR2_X1 U304 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n129) );
  OAI22_X1 U305 ( .A1(n392), .A2(n129), .B1(n105), .B2(n4), .ZN(n156) );
  XNOR2_X1 U306 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n150) );
  OAI22_X1 U307 ( .A1(n547), .A2(n150), .B1(n548), .B2(n106), .ZN(n155) );
  XNOR2_X1 U308 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n132) );
  XNOR2_X1 U309 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n111) );
  OAI22_X1 U310 ( .A1(n563), .A2(n132), .B1(n561), .B2(n111), .ZN(n127) );
  INV_X1 U311 ( .A(n127), .ZN(n154) );
  INV_X1 U312 ( .A(n107), .ZN(n140) );
  OAI22_X1 U313 ( .A1(n547), .A2(n109), .B1(n548), .B2(n108), .ZN(n139) );
  OAI22_X1 U314 ( .A1(n563), .A2(n111), .B1(n561), .B2(n110), .ZN(n128) );
  OAI22_X1 U315 ( .A1(n477), .A2(n113), .B1(n486), .B2(n112), .ZN(n126) );
  FA_X1 U316 ( .A(n116), .B(n115), .CI(n114), .CO(n117), .S(n120) );
  FA_X1 U317 ( .A(n119), .B(n118), .CI(n117), .CO(n1193), .S(n1196) );
  FA_X1 U318 ( .A(n122), .B(n121), .CI(n120), .CO(n1197), .S(n1200) );
  FA_X1 U319 ( .A(n125), .B(n124), .CI(n123), .CO(n142), .S(n169) );
  FA_X1 U320 ( .A(n128), .B(n127), .CI(n126), .CO(n138), .S(n168) );
  XNOR2_X1 U321 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n151) );
  OAI22_X1 U322 ( .A1(n392), .A2(n151), .B1(n129), .B2(n4), .ZN(n164) );
  XNOR2_X1 U323 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n160) );
  OAI22_X1 U324 ( .A1(n428), .A2(n160), .B1(n446), .B2(n130), .ZN(n163) );
  XNOR2_X1 U325 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n161) );
  OAI22_X1 U326 ( .A1(n509), .A2(n161), .B1(n131), .B2(n7), .ZN(n162) );
  XNOR2_X1 U327 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n133) );
  OAI22_X1 U328 ( .A1(n563), .A2(n133), .B1(n561), .B2(n132), .ZN(n158) );
  XNOR2_X1 U329 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n176) );
  OAI22_X1 U330 ( .A1(n563), .A2(n176), .B1(n561), .B2(n133), .ZN(n177) );
  XNOR2_X1 U331 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n153) );
  OAI22_X1 U332 ( .A1(n477), .A2(n153), .B1(n486), .B2(n134), .ZN(n157) );
  FA_X1 U333 ( .A(n137), .B(n136), .CI(n135), .CO(n166), .S(n192) );
  FA_X1 U334 ( .A(n140), .B(n139), .CI(n138), .CO(n121), .S(n145) );
  FA_X1 U335 ( .A(n143), .B(n142), .CI(n141), .CO(n122), .S(n144) );
  FA_X1 U336 ( .A(n146), .B(n145), .CI(n144), .CO(n1199), .S(n1203) );
  XNOR2_X2 U337 ( .A(xbuf_real_stage2[1]), .B(xbuf_real_stage2[2]), .ZN(n531)
         );
  OR2_X1 U338 ( .A1(n159), .A2(n529), .ZN(n149) );
  OR2_X1 U339 ( .A1(n159), .A2(n531), .ZN(n148) );
  XNOR2_X1 U340 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n152) );
  OAI22_X1 U341 ( .A1(n547), .A2(n152), .B1(n548), .B2(n150), .ZN(n174) );
  XNOR2_X1 U342 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n179) );
  OAI22_X1 U343 ( .A1(n392), .A2(n179), .B1(n151), .B2(n4), .ZN(n191) );
  XNOR2_X1 U344 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n187) );
  OAI22_X1 U345 ( .A1(n547), .A2(n187), .B1(n548), .B2(n152), .ZN(n190) );
  XNOR2_X1 U346 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n178) );
  OAI22_X1 U347 ( .A1(n477), .A2(n178), .B1(n486), .B2(n153), .ZN(n189) );
  FA_X1 U348 ( .A(n156), .B(n155), .CI(n154), .CO(n165), .S(n196) );
  FA_X1 U349 ( .A(n158), .B(n177), .CI(n157), .CO(n193), .S(n222) );
  XNOR2_X1 U350 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n180) );
  OAI22_X1 U351 ( .A1(n529), .A2(n180), .B1(n159), .B2(n531), .ZN(n183) );
  XNOR2_X1 U352 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n175) );
  XNOR2_X1 U353 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n188) );
  OAI22_X1 U354 ( .A1(n509), .A2(n188), .B1(n161), .B2(n7), .ZN(n181) );
  FA_X1 U355 ( .A(n164), .B(n163), .CI(n162), .CO(n194), .S(n220) );
  FA_X1 U356 ( .A(n166), .B(n34), .CI(n165), .CO(n141), .S(n171) );
  FA_X1 U357 ( .A(n169), .B(n168), .CI(n167), .CO(n146), .S(n170) );
  FA_X1 U358 ( .A(n172), .B(n171), .CI(n170), .CO(n1202), .S(n1206) );
  FA_X1 U359 ( .A(n36), .B(n174), .CI(n173), .CO(n197), .S(n225) );
  XNOR2_X1 U360 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n215) );
  OAI22_X1 U361 ( .A1(n428), .A2(n215), .B1(n446), .B2(n175), .ZN(n205) );
  XNOR2_X1 U362 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n207) );
  OAI22_X1 U363 ( .A1(n563), .A2(n207), .B1(n561), .B2(n176), .ZN(n204) );
  OR2_X1 U364 ( .A1(n205), .A2(n204), .ZN(n203) );
  INV_X1 U365 ( .A(n177), .ZN(n202) );
  XNOR2_X1 U366 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n210) );
  OAI22_X1 U367 ( .A1(n477), .A2(n210), .B1(n486), .B2(n178), .ZN(n219) );
  XNOR2_X1 U368 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n208) );
  OAI22_X1 U369 ( .A1(n392), .A2(n208), .B1(n179), .B2(n4), .ZN(n218) );
  XNOR2_X1 U370 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n214) );
  FA_X1 U371 ( .A(n183), .B(n182), .CI(n181), .CO(n221), .S(n250) );
  XNOR2_X1 U372 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n206) );
  NAND2_X1 U373 ( .A1(xbuf_real_stage2[0]), .A2(n184), .ZN(n185) );
  OAI21_X1 U374 ( .B1(n533), .B2(n206), .A(n185), .ZN(n186) );
  INV_X1 U375 ( .A(n186), .ZN(n213) );
  XNOR2_X1 U376 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n209) );
  OAI22_X1 U377 ( .A1(n547), .A2(n209), .B1(n548), .B2(n187), .ZN(n212) );
  XNOR2_X1 U378 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n216) );
  OAI22_X1 U379 ( .A1(n509), .A2(n216), .B1(n188), .B2(n7), .ZN(n211) );
  FA_X1 U380 ( .A(n191), .B(n190), .CI(n189), .CO(n173), .S(n248) );
  FA_X1 U381 ( .A(n194), .B(n193), .CI(n192), .CO(n167), .S(n199) );
  FA_X1 U382 ( .A(n197), .B(n196), .CI(n195), .CO(n172), .S(n198) );
  FA_X1 U383 ( .A(n200), .B(n199), .CI(n198), .CO(n1205), .S(n1209) );
  FA_X1 U384 ( .A(n203), .B(n202), .CI(n201), .CO(n224), .S(n253) );
  XNOR2_X1 U385 ( .A(n205), .B(n204), .ZN(n231) );
  XNOR2_X1 U386 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n234) );
  OAI22_X1 U387 ( .A1(n533), .A2(n234), .B1(n206), .B2(n3), .ZN(n233) );
  OAI22_X1 U388 ( .A1(n563), .A2(n235), .B1(n561), .B2(n207), .ZN(n232) );
  XNOR2_X1 U389 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n236) );
  OAI22_X1 U390 ( .A1(n392), .A2(n236), .B1(n208), .B2(n4), .ZN(n247) );
  XNOR2_X1 U391 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n237) );
  OAI22_X1 U392 ( .A1(n547), .A2(n237), .B1(n548), .B2(n209), .ZN(n246) );
  XNOR2_X1 U393 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n238) );
  OAI22_X1 U394 ( .A1(n477), .A2(n238), .B1(n486), .B2(n210), .ZN(n245) );
  FA_X1 U395 ( .A(n213), .B(n212), .CI(n211), .CO(n249), .S(n279) );
  XNOR2_X1 U396 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n242) );
  OAI22_X1 U397 ( .A1(n529), .A2(n242), .B1(n214), .B2(n531), .ZN(n241) );
  OAI22_X1 U398 ( .A1(n428), .A2(n243), .B1(n446), .B2(n215), .ZN(n240) );
  XNOR2_X1 U399 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n244) );
  OAI22_X1 U400 ( .A1(n509), .A2(n244), .B1(n216), .B2(n7), .ZN(n239) );
  FA_X1 U401 ( .A(n219), .B(n218), .CI(n217), .CO(n201), .S(n277) );
  FA_X1 U402 ( .A(n222), .B(n221), .CI(n220), .CO(n195), .S(n227) );
  FA_X1 U403 ( .A(n225), .B(n224), .CI(n223), .CO(n200), .S(n226) );
  FA_X1 U404 ( .A(n228), .B(n227), .CI(n226), .CO(n1208), .S(n1212) );
  FA_X1 U405 ( .A(n231), .B(n230), .CI(n229), .CO(n252), .S(n282) );
  HA_X1 U406 ( .A(n233), .B(n232), .CO(n230), .S(n259) );
  XNOR2_X1 U407 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n266) );
  OAI22_X1 U408 ( .A1(n533), .A2(n266), .B1(n234), .B2(n3), .ZN(n261) );
  XNOR2_X1 U409 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n262) );
  OAI22_X1 U410 ( .A1(n563), .A2(n262), .B1(n561), .B2(n235), .ZN(n260) );
  XNOR2_X1 U411 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n271) );
  OAI22_X1 U412 ( .A1(n392), .A2(n271), .B1(n236), .B2(n4), .ZN(n276) );
  XNOR2_X1 U413 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n295) );
  OAI22_X1 U414 ( .A1(n547), .A2(n295), .B1(n548), .B2(n237), .ZN(n275) );
  XNOR2_X1 U415 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n265) );
  OAI22_X1 U416 ( .A1(n477), .A2(n265), .B1(n486), .B2(n238), .ZN(n274) );
  FA_X1 U417 ( .A(n241), .B(n240), .CI(n239), .CO(n278), .S(n307) );
  XNOR2_X1 U418 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n273) );
  OAI22_X1 U419 ( .A1(n529), .A2(n273), .B1(n242), .B2(n531), .ZN(n270) );
  XNOR2_X1 U420 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n272) );
  OAI22_X1 U421 ( .A1(n428), .A2(n272), .B1(n446), .B2(n243), .ZN(n269) );
  XNOR2_X1 U422 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n267) );
  OAI22_X1 U423 ( .A1(n509), .A2(n267), .B1(n244), .B2(n7), .ZN(n268) );
  FA_X1 U424 ( .A(n247), .B(n246), .CI(n245), .CO(n229), .S(n305) );
  FA_X1 U425 ( .A(n250), .B(n249), .CI(n248), .CO(n223), .S(n255) );
  FA_X1 U426 ( .A(n253), .B(n252), .CI(n251), .CO(n228), .S(n254) );
  FA_X1 U427 ( .A(n256), .B(n255), .CI(n254), .CO(n1211), .S(n1215) );
  FA_X1 U428 ( .A(n259), .B(n258), .CI(n257), .CO(n281), .S(n310) );
  HA_X1 U429 ( .A(n261), .B(n260), .CO(n258), .S(n304) );
  XNOR2_X1 U430 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n263) );
  OAI22_X1 U431 ( .A1(n563), .A2(n263), .B1(n561), .B2(n262), .ZN(n294) );
  XNOR2_X1 U432 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n286) );
  OAI22_X1 U433 ( .A1(n477), .A2(n286), .B1(n486), .B2(n265), .ZN(n292) );
  XNOR2_X1 U434 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[14]), 
        .ZN(n297) );
  OAI22_X1 U435 ( .A1(n533), .A2(n297), .B1(n266), .B2(n3), .ZN(n291) );
  XNOR2_X1 U436 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n289) );
  OAI22_X1 U437 ( .A1(n509), .A2(n289), .B1(n267), .B2(n7), .ZN(n290) );
  FA_X1 U438 ( .A(n270), .B(n269), .CI(n268), .CO(n306), .S(n332) );
  XNOR2_X1 U439 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n287) );
  OAI22_X1 U440 ( .A1(n392), .A2(n287), .B1(n271), .B2(n4), .ZN(n326) );
  OAI22_X1 U441 ( .A1(n428), .A2(n300), .B1(n446), .B2(n272), .ZN(n325) );
  XNOR2_X1 U442 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n288) );
  OAI22_X1 U443 ( .A1(n529), .A2(n288), .B1(n273), .B2(n531), .ZN(n324) );
  FA_X1 U444 ( .A(n276), .B(n275), .CI(n274), .CO(n257), .S(n330) );
  FA_X1 U445 ( .A(n279), .B(n278), .CI(n277), .CO(n251), .S(n284) );
  FA_X1 U446 ( .A(n282), .B(n281), .CI(n280), .CO(n256), .S(n283) );
  FA_X1 U447 ( .A(n285), .B(n284), .CI(n283), .CO(n1214), .S(n1218) );
  XNOR2_X1 U448 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n320) );
  OAI22_X1 U449 ( .A1(n477), .A2(n320), .B1(n486), .B2(n286), .ZN(n344) );
  XNOR2_X1 U450 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n318) );
  XNOR2_X1 U451 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n350) );
  XNOR2_X1 U452 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n301) );
  XNOR2_X1 U453 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[12]), 
        .ZN(n348) );
  XNOR2_X1 U454 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[13]), 
        .ZN(n298) );
  OAI22_X1 U455 ( .A1(n533), .A2(n348), .B1(n298), .B2(n3), .ZN(n351) );
  XNOR2_X1 U456 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n314) );
  OAI22_X1 U457 ( .A1(n529), .A2(n314), .B1(n288), .B2(n531), .ZN(n347) );
  XNOR2_X1 U458 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n315) );
  XNOR2_X1 U459 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n296) );
  XNOR2_X1 U460 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n317) );
  OAI22_X1 U461 ( .A1(n509), .A2(n317), .B1(n289), .B2(n7), .ZN(n345) );
  FA_X1 U462 ( .A(n292), .B(n291), .CI(n290), .CO(n302), .S(n339) );
  HA_X1 U463 ( .A(n294), .B(n293), .CO(n303), .S(n329) );
  OAI22_X1 U464 ( .A1(n547), .A2(n296), .B1(n548), .B2(n295), .ZN(n328) );
  OAI22_X1 U465 ( .A1(n533), .A2(n298), .B1(n297), .B2(n3), .ZN(n323) );
  INV_X1 U466 ( .A(n561), .ZN(n299) );
  AND2_X1 U467 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n299), .ZN(n322) );
  FA_X1 U468 ( .A(n304), .B(n303), .CI(n302), .CO(n309), .S(n333) );
  FA_X1 U469 ( .A(n307), .B(n306), .CI(n305), .CO(n280), .S(n312) );
  FA_X1 U470 ( .A(n310), .B(n309), .CI(n308), .CO(n285), .S(n311) );
  FA_X1 U471 ( .A(n313), .B(n312), .CI(n311), .CO(n1217), .S(n1221) );
  XNOR2_X1 U472 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n353) );
  OAI22_X1 U473 ( .A1(n529), .A2(n353), .B1(n314), .B2(n531), .ZN(n367) );
  XNOR2_X1 U474 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n316) );
  XNOR2_X1 U475 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n355) );
  OR2_X1 U476 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n10), .ZN(n319) );
  XNOR2_X1 U477 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n373) );
  FA_X1 U478 ( .A(n323), .B(n322), .CI(n321), .CO(n327), .S(n362) );
  FA_X1 U479 ( .A(n326), .B(n325), .CI(n324), .CO(n331), .S(n357) );
  FA_X1 U480 ( .A(n329), .B(n328), .CI(n327), .CO(n334), .S(n356) );
  FA_X1 U481 ( .A(n332), .B(n331), .CI(n330), .CO(n308), .S(n337) );
  FA_X1 U482 ( .A(n335), .B(n334), .CI(n333), .CO(n313), .S(n336) );
  FA_X1 U483 ( .A(n338), .B(n337), .CI(n336), .CO(n1220), .S(n1224) );
  FA_X1 U484 ( .A(n341), .B(n340), .CI(n339), .CO(n335), .S(n361) );
  FA_X1 U485 ( .A(n344), .B(n343), .CI(n342), .CO(n341), .S(n379) );
  FA_X1 U486 ( .A(n347), .B(n346), .CI(n345), .CO(n340), .S(n378) );
  XNOR2_X1 U487 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[11]), 
        .ZN(n372) );
  OAI22_X1 U488 ( .A1(n533), .A2(n372), .B1(n348), .B2(n3), .ZN(n388) );
  INV_X1 U489 ( .A(n548), .ZN(n349) );
  AND2_X1 U490 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n349), .ZN(n387) );
  XNOR2_X1 U491 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n371) );
  HA_X1 U492 ( .A(n352), .B(n351), .CO(n342), .S(n396) );
  XNOR2_X1 U493 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n374) );
  XNOR2_X1 U494 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n390) );
  XNOR2_X1 U495 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n376) );
  FA_X1 U496 ( .A(n358), .B(n357), .CI(n356), .CO(n338), .S(n359) );
  FA_X1 U497 ( .A(n361), .B(n360), .CI(n359), .CO(n1223), .S(n1227) );
  FA_X1 U498 ( .A(n364), .B(n363), .CI(n362), .CO(n358), .S(n382) );
  FA_X1 U499 ( .A(n367), .B(n366), .CI(n365), .CO(n364), .S(n400) );
  FA_X1 U500 ( .A(n370), .B(n369), .CI(n368), .CO(n363), .S(n399) );
  XNOR2_X1 U501 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[10]), 
        .ZN(n410) );
  OAI22_X1 U502 ( .A1(n533), .A2(n410), .B1(n372), .B2(n3), .ZN(n393) );
  XNOR2_X1 U503 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n389) );
  XNOR2_X1 U504 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n407) );
  OR2_X1 U505 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n11), .ZN(n375) );
  XNOR2_X1 U506 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n409) );
  FA_X1 U507 ( .A(n379), .B(n378), .CI(n377), .CO(n360), .S(n380) );
  FA_X1 U508 ( .A(n382), .B(n381), .CI(n380), .CO(n1226), .S(n1230) );
  FA_X1 U509 ( .A(n385), .B(n384), .CI(n383), .CO(n395), .S(n419) );
  FA_X1 U510 ( .A(n388), .B(n387), .CI(n386), .CO(n397), .S(n418) );
  XNOR2_X1 U511 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n408) );
  XNOR2_X1 U512 ( .A(xbuf_real_stage2[11]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n391) );
  HA_X1 U513 ( .A(n394), .B(n393), .CO(n406), .S(n433) );
  FA_X1 U514 ( .A(n397), .B(n396), .CI(n395), .CO(n377), .S(n402) );
  FA_X1 U515 ( .A(n400), .B(n399), .CI(n398), .CO(n381), .S(n401) );
  FA_X1 U516 ( .A(n403), .B(n402), .CI(n401), .CO(n1229), .S(n1233) );
  FA_X1 U517 ( .A(n406), .B(n405), .CI(n404), .CO(n398), .S(n422) );
  XNOR2_X1 U518 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n423) );
  XNOR2_X1 U519 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n444) );
  XNOR2_X1 U520 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[9]), 
        .ZN(n429) );
  OAI22_X1 U521 ( .A1(n533), .A2(n429), .B1(n410), .B2(n3), .ZN(n432) );
  INV_X1 U522 ( .A(n4), .ZN(n412) );
  XNOR2_X1 U523 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n424) );
  FA_X1 U524 ( .A(n416), .B(n415), .CI(n414), .CO(n404), .S(n436) );
  FA_X1 U525 ( .A(n419), .B(n418), .CI(n417), .CO(n403), .S(n420) );
  FA_X1 U526 ( .A(n422), .B(n421), .CI(n420), .CO(n1232), .S(n1236) );
  XNOR2_X1 U527 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[6]), 
        .ZN(n462) );
  XNOR2_X1 U528 ( .A(xbuf_real_stage2[9]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n425) );
  XNOR2_X1 U529 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n461) );
  OR2_X1 U530 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n12), .ZN(n427) );
  XNOR2_X1 U531 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[8]), 
        .ZN(n445) );
  FA_X1 U532 ( .A(n432), .B(n431), .CI(n430), .CO(n437), .S(n452) );
  FA_X1 U533 ( .A(n435), .B(n434), .CI(n433), .CO(n417), .S(n440) );
  FA_X1 U534 ( .A(n438), .B(n437), .CI(n436), .CO(n421), .S(n439) );
  FA_X1 U535 ( .A(n441), .B(n440), .CI(n439), .CO(n1235), .S(n1239) );
  HA_X1 U536 ( .A(n443), .B(n442), .CO(n453), .S(n467) );
  XNOR2_X1 U537 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n448) );
  XNOR2_X1 U538 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[7]), 
        .ZN(n464) );
  OAI22_X1 U539 ( .A1(n533), .A2(n464), .B1(n445), .B2(n3), .ZN(n473) );
  INV_X1 U540 ( .A(n446), .ZN(n447) );
  XNOR2_X1 U541 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n475) );
  FA_X1 U542 ( .A(n451), .B(n450), .CI(n449), .CO(n438), .S(n456) );
  FA_X1 U543 ( .A(n454), .B(n453), .CI(n452), .CO(n441), .S(n455) );
  FA_X1 U544 ( .A(n457), .B(n456), .CI(n455), .CO(n1238), .S(n1242) );
  FA_X1 U545 ( .A(n460), .B(n459), .CI(n458), .CO(n454), .S(n470) );
  XNOR2_X1 U546 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n478) );
  XNOR2_X1 U547 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n474) );
  OR2_X1 U548 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n13), .ZN(n463) );
  FA_X1 U549 ( .A(n467), .B(n466), .CI(n465), .CO(n457), .S(n468) );
  FA_X1 U550 ( .A(n470), .B(n469), .CI(n468), .CO(n1241), .S(n1245) );
  FA_X1 U551 ( .A(n473), .B(n472), .CI(n471), .CO(n465), .S(n484) );
  XNOR2_X1 U552 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n488) );
  XNOR2_X1 U553 ( .A(xbuf_real_stage2[7]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n476) );
  XNOR2_X1 U554 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n499) );
  FA_X1 U555 ( .A(n481), .B(n480), .CI(n479), .CO(n469), .S(n482) );
  FA_X1 U556 ( .A(n484), .B(n483), .CI(n482), .CO(n1244), .S(n1248) );
  XNOR2_X1 U557 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[5]), 
        .ZN(n498) );
  OAI22_X1 U558 ( .A1(n533), .A2(n498), .B1(n485), .B2(n3), .ZN(n502) );
  INV_X1 U559 ( .A(n486), .ZN(n487) );
  AND2_X1 U560 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n487), .ZN(n501) );
  XNOR2_X1 U561 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n497) );
  HA_X1 U562 ( .A(n490), .B(n489), .CO(n479), .S(n495) );
  FA_X1 U563 ( .A(n493), .B(n492), .CI(n491), .CO(n483), .S(n494) );
  FA_X1 U564 ( .A(n496), .B(n495), .CI(n494), .CO(n1247), .S(n1251) );
  XNOR2_X1 U565 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[2]), 
        .ZN(n518) );
  XNOR2_X1 U566 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[4]), 
        .ZN(n515) );
  OAI22_X1 U567 ( .A1(n533), .A2(n515), .B1(n498), .B2(n3), .ZN(n510) );
  XNOR2_X1 U568 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n507) );
  FA_X1 U569 ( .A(n502), .B(n501), .CI(n500), .CO(n496), .S(n503) );
  FA_X1 U570 ( .A(n505), .B(n504), .CI(n503), .CO(n1250), .S(n1254) );
  OR2_X1 U571 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n14), .ZN(n506) );
  XNOR2_X1 U572 ( .A(xbuf_real_stage2[5]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n508) );
  HA_X1 U573 ( .A(n511), .B(n510), .CO(n505), .S(n512) );
  FA_X1 U574 ( .A(n514), .B(n513), .CI(n512), .CO(n1253), .S(n1257) );
  XNOR2_X1 U575 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[3]), 
        .ZN(n525) );
  OAI22_X1 U576 ( .A1(n533), .A2(n525), .B1(n515), .B2(n3), .ZN(n521) );
  INV_X1 U577 ( .A(n516), .ZN(n517) );
  AND2_X1 U578 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n517), .ZN(n520) );
  XNOR2_X1 U579 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n522) );
  OAI22_X1 U580 ( .A1(n529), .A2(n522), .B1(n518), .B2(n531), .ZN(n519) );
  FA_X1 U581 ( .A(n521), .B(n520), .CI(n519), .CO(n1256), .S(n1260) );
  XNOR2_X1 U582 ( .A(xbuf_real_stage2[3]), .B(twiddle_real_m_imag_stage2[0]), 
        .ZN(n524) );
  OAI21_X1 U583 ( .B1(n524), .B2(n529), .A(n523), .ZN(n527) );
  OAI22_X1 U584 ( .A1(n533), .A2(n530), .B1(n525), .B2(n3), .ZN(n526) );
  HA_X1 U585 ( .A(n527), .B(n526), .CO(n1259), .S(n1263) );
  OR2_X1 U586 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n32), .ZN(n528) );
  OAI22_X1 U587 ( .A1(n529), .A2(n32), .B1(n528), .B2(n531), .ZN(n1262) );
  XNOR2_X1 U588 ( .A(xbuf_real_stage2[1]), .B(twiddle_real_m_imag_stage2[1]), 
        .ZN(n532) );
  OAI22_X1 U589 ( .A1(n533), .A2(n532), .B1(n530), .B2(n3), .ZN(n1266) );
  AND2_X1 U590 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n147), .ZN(n1265) );
  OAI22_X1 U591 ( .A1(n533), .A2(twiddle_real_m_imag_stage2[0]), .B1(n532), 
        .B2(n3), .ZN(n1268) );
  NAND2_X1 U592 ( .A1(n534), .A2(n533), .ZN(n1267) );
  NAND2_X1 U593 ( .A1(n1268), .A2(n1267), .ZN(n1269) );
  INV_X1 U594 ( .A(n1269), .ZN(n1264) );
  FA_X1 U595 ( .A(n536), .B(n31), .CI(n535), .CO(n554), .S(n1188) );
  XNOR2_X1 U596 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[15]), 
        .ZN(n546) );
  OAI22_X1 U597 ( .A1(n563), .A2(n537), .B1(n561), .B2(n546), .ZN(n558) );
  INV_X1 U598 ( .A(n558), .ZN(n545) );
  XNOR2_X1 U599 ( .A(xbuf_real_stage2[13]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n549) );
  OAI22_X1 U600 ( .A1(n547), .A2(n538), .B1(n548), .B2(n549), .ZN(n544) );
  FA_X1 U601 ( .A(n541), .B(n540), .CI(n539), .CO(n543), .S(n535) );
  FA_X1 U602 ( .A(n545), .B(n544), .CI(n543), .CO(n1185), .S(n555) );
  XNOR2_X1 U603 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[16]), 
        .ZN(n562) );
  OAI22_X1 U604 ( .A1(n563), .A2(n546), .B1(n561), .B2(n562), .ZN(n559) );
  OR2_X1 U605 ( .A1(n549), .A2(n547), .ZN(n551) );
  OR2_X1 U606 ( .A1(n549), .A2(n548), .ZN(n550) );
  NAND2_X1 U607 ( .A1(n553), .A2(n552), .ZN(n557) );
  NAND2_X1 U608 ( .A1(n555), .A2(n554), .ZN(n556) );
  NAND2_X1 U609 ( .A1(n557), .A2(n556), .ZN(n1183) );
  FA_X1 U610 ( .A(n559), .B(n558), .CI(n28), .CO(n566), .S(n1184) );
  XNOR2_X1 U611 ( .A(xbuf_real_stage2[15]), .B(twiddle_real_m_imag_stage2[17]), 
        .ZN(n560) );
  OAI22_X1 U612 ( .A1(n563), .A2(n562), .B1(n561), .B2(n560), .ZN(n564) );
  INV_X1 U613 ( .A(n564), .ZN(n565) );
  XOR2_X1 U614 ( .A(n566), .B(n565), .Z(n567) );
  XOR2_X1 U615 ( .A(n568), .B(n567), .Z(tmp_i[31]) );
  XNOR2_X1 U616 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n582) );
  XNOR2_X1 U617 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n572) );
  OAI22_X1 U618 ( .A1(n918), .A2(n582), .B1(n572), .B2(n5), .ZN(n579) );
  XNOR2_X2 U619 ( .A(xbuf_imag_stage2[11]), .B(xbuf_imag_stage2[12]), .ZN(
        n1075) );
  XNOR2_X1 U620 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n586) );
  XNOR2_X1 U621 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n576) );
  OAI22_X1 U622 ( .A1(n1074), .A2(n586), .B1(n1075), .B2(n576), .ZN(n578) );
  XNOR2_X2 U623 ( .A(xbuf_imag_stage2[13]), .B(xbuf_imag_stage2[14]), .ZN(
        n1088) );
  XNOR2_X1 U624 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n580) );
  OAI22_X1 U625 ( .A1(n1090), .A2(n580), .B1(n1088), .B2(n575), .ZN(n1067) );
  INV_X1 U626 ( .A(n1067), .ZN(n577) );
  OR2_X1 U627 ( .A1(n572), .A2(n918), .ZN(n574) );
  OR2_X1 U628 ( .A1(n572), .A2(n5), .ZN(n573) );
  XNOR2_X1 U629 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n1064) );
  OAI22_X1 U630 ( .A1(n1090), .A2(n575), .B1(n1088), .B2(n1064), .ZN(n1068) );
  XNOR2_X1 U631 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n1065) );
  FA_X1 U632 ( .A(n579), .B(n578), .CI(n577), .CO(n1063), .S(n592) );
  XNOR2_X1 U633 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n581) );
  OAI22_X1 U634 ( .A1(n1090), .A2(n581), .B1(n1088), .B2(n580), .ZN(n603) );
  XNOR2_X1 U635 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n596) );
  OAI22_X1 U636 ( .A1(n1090), .A2(n596), .B1(n1088), .B2(n581), .ZN(n602) );
  OAI22_X1 U637 ( .A1(n918), .A2(n589), .B1(n582), .B2(n5), .ZN(n601) );
  XNOR2_X1 U638 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n588) );
  XNOR2_X2 U639 ( .A(xbuf_imag_stage2[7]), .B(xbuf_imag_stage2[8]), .ZN(n987)
         );
  OR2_X1 U640 ( .A1(n588), .A2(n970), .ZN(n585) );
  OR2_X1 U641 ( .A1(n588), .A2(n987), .ZN(n584) );
  XNOR2_X1 U642 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n587) );
  OAI22_X1 U643 ( .A1(n1074), .A2(n587), .B1(n1075), .B2(n586), .ZN(n605) );
  XNOR2_X1 U644 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n595) );
  OAI22_X1 U645 ( .A1(n1074), .A2(n595), .B1(n1075), .B2(n587), .ZN(n614) );
  XNOR2_X1 U646 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n593) );
  OAI22_X1 U647 ( .A1(n970), .A2(n593), .B1(n987), .B2(n588), .ZN(n613) );
  XNOR2_X1 U648 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n594) );
  OAI22_X1 U649 ( .A1(n918), .A2(n594), .B1(n589), .B2(n5), .ZN(n612) );
  FA_X1 U650 ( .A(n592), .B(n591), .CI(n590), .CO(n1100), .S(n1104) );
  XNOR2_X1 U651 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n618) );
  OAI22_X1 U652 ( .A1(n970), .A2(n618), .B1(n987), .B2(n593), .ZN(n616) );
  XNOR2_X1 U653 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n628) );
  XNOR2_X1 U654 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n597) );
  OAI22_X1 U655 ( .A1(n1090), .A2(n628), .B1(n1088), .B2(n597), .ZN(n637) );
  XNOR2_X1 U656 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n617) );
  OAI22_X1 U657 ( .A1(n918), .A2(n617), .B1(n594), .B2(n5), .ZN(n615) );
  INV_X1 U658 ( .A(n602), .ZN(n610) );
  XNOR2_X1 U659 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n638) );
  OAI22_X1 U660 ( .A1(n1074), .A2(n638), .B1(n1075), .B2(n595), .ZN(n621) );
  OAI22_X1 U661 ( .A1(n1090), .A2(n597), .B1(n1088), .B2(n596), .ZN(n620) );
  XNOR2_X1 U662 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n619) );
  XNOR2_X2 U663 ( .A(xbuf_imag_stage2[5]), .B(xbuf_imag_stage2[6]), .ZN(n1025)
         );
  OR2_X1 U664 ( .A1(n619), .A2(n1001), .ZN(n600) );
  OR2_X1 U665 ( .A1(n619), .A2(n1025), .ZN(n599) );
  FA_X1 U666 ( .A(n603), .B(n602), .CI(n601), .CO(n591), .S(n607) );
  FA_X1 U667 ( .A(n26), .B(n605), .CI(n604), .CO(n590), .S(n606) );
  FA_X1 U668 ( .A(n608), .B(n607), .CI(n606), .CO(n1103), .S(n1107) );
  FA_X1 U669 ( .A(n611), .B(n610), .CI(n609), .CO(n608), .S(n624) );
  FA_X1 U670 ( .A(n614), .B(n613), .CI(n612), .CO(n604), .S(n623) );
  FA_X1 U671 ( .A(n616), .B(n637), .CI(n615), .CO(n611), .S(n646) );
  XNOR2_X1 U672 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n629) );
  XNOR2_X1 U673 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n640) );
  OAI22_X1 U674 ( .A1(n970), .A2(n640), .B1(n987), .B2(n618), .ZN(n626) );
  XNOR2_X1 U675 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n642) );
  OAI22_X1 U676 ( .A1(n1001), .A2(n642), .B1(n1025), .B2(n619), .ZN(n625) );
  FA_X1 U677 ( .A(n621), .B(n620), .CI(n25), .CO(n609), .S(n644) );
  FA_X1 U678 ( .A(n624), .B(n623), .CI(n622), .CO(n1106), .S(n1110) );
  FA_X1 U679 ( .A(n627), .B(n626), .CI(n625), .CO(n645), .S(n670) );
  XNOR2_X1 U680 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n635) );
  XNOR2_X1 U681 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n639) );
  OAI22_X1 U682 ( .A1(n1074), .A2(n635), .B1(n1075), .B2(n639), .ZN(n652) );
  OAI22_X1 U683 ( .A1(n1090), .A2(n636), .B1(n1088), .B2(n628), .ZN(n651) );
  XNOR2_X1 U684 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n634) );
  OAI22_X1 U685 ( .A1(n918), .A2(n634), .B1(n629), .B2(n5), .ZN(n650) );
  XNOR2_X1 U686 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n658) );
  XNOR2_X1 U687 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n643) );
  OAI22_X1 U688 ( .A1(n1001), .A2(n658), .B1(n1025), .B2(n643), .ZN(n664) );
  XNOR2_X1 U689 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n659) );
  XNOR2_X1 U690 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n641) );
  OAI22_X1 U691 ( .A1(n970), .A2(n659), .B1(n987), .B2(n641), .ZN(n663) );
  XNOR2_X1 U692 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n661) );
  XNOR2_X1 U693 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n631) );
  OAI22_X1 U694 ( .A1(n1035), .A2(n661), .B1(n6), .B2(n631), .ZN(n662) );
  OR2_X1 U695 ( .A1(n631), .A2(n1035), .ZN(n633) );
  OR2_X1 U696 ( .A1(n631), .A2(n6), .ZN(n632) );
  XNOR2_X1 U697 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n677) );
  OAI22_X1 U698 ( .A1(n918), .A2(n677), .B1(n634), .B2(n5), .ZN(n683) );
  XNOR2_X1 U699 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n656) );
  OAI22_X1 U700 ( .A1(n1074), .A2(n656), .B1(n1075), .B2(n635), .ZN(n682) );
  XNOR2_X1 U701 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n657) );
  OAI22_X1 U702 ( .A1(n1090), .A2(n657), .B1(n1088), .B2(n636), .ZN(n654) );
  INV_X1 U703 ( .A(n637), .ZN(n667) );
  OAI22_X1 U704 ( .A1(n970), .A2(n641), .B1(n987), .B2(n640), .ZN(n655) );
  OAI22_X1 U705 ( .A1(n1001), .A2(n643), .B1(n1025), .B2(n642), .ZN(n653) );
  FA_X1 U706 ( .A(n646), .B(n645), .CI(n644), .CO(n622), .S(n647) );
  FA_X1 U707 ( .A(n649), .B(n648), .CI(n647), .CO(n1109), .S(n1113) );
  FA_X1 U708 ( .A(n652), .B(n651), .CI(n650), .CO(n669), .S(n696) );
  FA_X1 U709 ( .A(n655), .B(n654), .CI(n653), .CO(n665), .S(n695) );
  XNOR2_X1 U710 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n687) );
  OAI22_X1 U711 ( .A1(n1074), .A2(n687), .B1(n1075), .B2(n656), .ZN(n691) );
  XNOR2_X1 U712 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n660) );
  OAI22_X1 U713 ( .A1(n1090), .A2(n660), .B1(n1088), .B2(n657), .ZN(n690) );
  XNOR2_X1 U714 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n678) );
  OAI22_X1 U715 ( .A1(n1001), .A2(n678), .B1(n1025), .B2(n658), .ZN(n689) );
  XNOR2_X1 U716 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n679) );
  OAI22_X1 U717 ( .A1(n970), .A2(n679), .B1(n987), .B2(n659), .ZN(n685) );
  XNOR2_X1 U718 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n703) );
  OAI22_X1 U719 ( .A1(n1090), .A2(n703), .B1(n1088), .B2(n660), .ZN(n704) );
  XNOR2_X1 U720 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n688) );
  OAI22_X1 U721 ( .A1(n1035), .A2(n688), .B1(n6), .B2(n661), .ZN(n684) );
  FA_X1 U722 ( .A(n664), .B(n663), .CI(n662), .CO(n693), .S(n719) );
  FA_X1 U723 ( .A(n667), .B(n666), .CI(n665), .CO(n648), .S(n672) );
  FA_X1 U724 ( .A(n670), .B(n669), .CI(n668), .CO(n649), .S(n671) );
  FA_X1 U725 ( .A(n673), .B(n672), .CI(n671), .CO(n1112), .S(n1116) );
  XNOR2_X1 U726 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n680) );
  XNOR2_X2 U727 ( .A(xbuf_imag_stage2[1]), .B(xbuf_imag_stage2[2]), .ZN(n1057)
         );
  OR2_X1 U728 ( .A1(n680), .A2(n1057), .ZN(n675) );
  XNOR2_X1 U729 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n686) );
  OAI22_X1 U730 ( .A1(n918), .A2(n686), .B1(n677), .B2(n5), .ZN(n701) );
  OAI22_X1 U731 ( .A1(n1001), .A2(n705), .B1(n1025), .B2(n678), .ZN(n710) );
  XNOR2_X1 U732 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n702) );
  OAI22_X1 U733 ( .A1(n970), .A2(n702), .B1(n987), .B2(n679), .ZN(n709) );
  XNOR2_X1 U734 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n711) );
  OAI22_X1 U735 ( .A1(n1055), .A2(n711), .B1(n680), .B2(n1057), .ZN(n708) );
  FA_X1 U736 ( .A(n683), .B(n682), .CI(n681), .CO(n692), .S(n723) );
  FA_X1 U737 ( .A(n685), .B(n704), .CI(n684), .CO(n720), .S(n749) );
  XNOR2_X1 U738 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n706) );
  XNOR2_X1 U739 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n712) );
  OAI22_X1 U740 ( .A1(n1074), .A2(n712), .B1(n1075), .B2(n687), .ZN(n717) );
  XNOR2_X1 U741 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n707) );
  OAI22_X1 U742 ( .A1(n1035), .A2(n707), .B1(n6), .B2(n688), .ZN(n716) );
  FA_X1 U743 ( .A(n691), .B(n690), .CI(n689), .CO(n721), .S(n747) );
  FA_X1 U744 ( .A(n693), .B(n24), .CI(n692), .CO(n668), .S(n698) );
  FA_X1 U745 ( .A(n696), .B(n695), .CI(n694), .CO(n673), .S(n697) );
  FA_X1 U746 ( .A(n699), .B(n698), .CI(n697), .CO(n1115), .S(n1119) );
  FA_X1 U747 ( .A(n41), .B(n701), .CI(n700), .CO(n724), .S(n752) );
  XNOR2_X1 U748 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n736) );
  XNOR2_X1 U749 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n734) );
  OAI22_X1 U750 ( .A1(n1090), .A2(n734), .B1(n1088), .B2(n703), .ZN(n731) );
  OR2_X1 U751 ( .A1(n732), .A2(n731), .ZN(n730) );
  INV_X1 U752 ( .A(n704), .ZN(n729) );
  XNOR2_X1 U753 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n735) );
  OAI22_X1 U754 ( .A1(n1001), .A2(n735), .B1(n1025), .B2(n705), .ZN(n746) );
  XNOR2_X1 U755 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n741) );
  XNOR2_X1 U756 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n743) );
  OAI22_X1 U757 ( .A1(n1035), .A2(n743), .B1(n6), .B2(n707), .ZN(n744) );
  FA_X1 U758 ( .A(n710), .B(n709), .CI(n708), .CO(n700), .S(n777) );
  XNOR2_X1 U759 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n737) );
  OAI22_X1 U760 ( .A1(n1055), .A2(n737), .B1(n711), .B2(n1057), .ZN(n740) );
  XNOR2_X1 U761 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n742) );
  OAI22_X1 U762 ( .A1(n1074), .A2(n742), .B1(n1075), .B2(n712), .ZN(n739) );
  XNOR2_X1 U763 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n733) );
  INV_X1 U764 ( .A(n733), .ZN(n713) );
  NAND2_X1 U765 ( .A1(xbuf_imag_stage2[0]), .A2(n713), .ZN(n714) );
  OAI21_X1 U766 ( .B1(n1061), .B2(n733), .A(n714), .ZN(n715) );
  INV_X1 U767 ( .A(n715), .ZN(n738) );
  FA_X1 U768 ( .A(n718), .B(n717), .CI(n716), .CO(n748), .S(n775) );
  FA_X1 U769 ( .A(n721), .B(n720), .CI(n719), .CO(n694), .S(n726) );
  FA_X1 U770 ( .A(n724), .B(n723), .CI(n722), .CO(n699), .S(n725) );
  FA_X1 U771 ( .A(n727), .B(n726), .CI(n725), .CO(n1118), .S(n1122) );
  FA_X1 U772 ( .A(n730), .B(n729), .CI(n728), .CO(n751), .S(n780) );
  XNOR2_X1 U773 ( .A(n732), .B(n731), .ZN(n758) );
  XNOR2_X1 U774 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n761) );
  OAI22_X1 U775 ( .A1(n1061), .A2(n761), .B1(n733), .B2(n2), .ZN(n760) );
  XNOR2_X1 U776 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n762) );
  OAI22_X1 U777 ( .A1(n1090), .A2(n762), .B1(n1088), .B2(n734), .ZN(n759) );
  XNOR2_X1 U778 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n763) );
  OAI22_X1 U779 ( .A1(n1001), .A2(n763), .B1(n1025), .B2(n735), .ZN(n768) );
  XNOR2_X1 U780 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n764) );
  OAI22_X1 U781 ( .A1(n970), .A2(n764), .B1(n987), .B2(n736), .ZN(n767) );
  XNOR2_X1 U782 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n765) );
  OAI22_X1 U783 ( .A1(n1055), .A2(n765), .B1(n737), .B2(n1057), .ZN(n766) );
  FA_X1 U784 ( .A(n740), .B(n739), .CI(n738), .CO(n776), .S(n805) );
  XNOR2_X1 U785 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n769) );
  OAI22_X1 U786 ( .A1(n918), .A2(n769), .B1(n741), .B2(n5), .ZN(n774) );
  XNOR2_X1 U787 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n770) );
  OAI22_X1 U788 ( .A1(n1074), .A2(n770), .B1(n1075), .B2(n742), .ZN(n773) );
  XNOR2_X1 U789 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n771) );
  OAI22_X1 U790 ( .A1(n1035), .A2(n771), .B1(n6), .B2(n743), .ZN(n772) );
  FA_X1 U791 ( .A(n746), .B(n745), .CI(n744), .CO(n728), .S(n803) );
  FA_X1 U792 ( .A(n749), .B(n748), .CI(n747), .CO(n722), .S(n754) );
  FA_X1 U793 ( .A(n752), .B(n751), .CI(n750), .CO(n727), .S(n753) );
  FA_X1 U794 ( .A(n755), .B(n754), .CI(n753), .CO(n1121), .S(n1125) );
  FA_X1 U795 ( .A(n758), .B(n757), .CI(n756), .CO(n779), .S(n808) );
  HA_X1 U796 ( .A(n760), .B(n759), .CO(n757), .S(n786) );
  XNOR2_X1 U797 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n790) );
  OAI22_X1 U798 ( .A1(n1061), .A2(n790), .B1(n761), .B2(n2), .ZN(n788) );
  XNOR2_X1 U799 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n823) );
  OAI22_X1 U800 ( .A1(n1090), .A2(n823), .B1(n1088), .B2(n762), .ZN(n787) );
  XNOR2_X1 U801 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n800) );
  OAI22_X1 U802 ( .A1(n1001), .A2(n800), .B1(n1025), .B2(n763), .ZN(n799) );
  XNOR2_X1 U803 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n789) );
  OAI22_X1 U804 ( .A1(n970), .A2(n789), .B1(n987), .B2(n764), .ZN(n798) );
  OAI22_X1 U805 ( .A1(n1055), .A2(n793), .B1(n765), .B2(n1057), .ZN(n797) );
  FA_X1 U806 ( .A(n768), .B(n767), .CI(n766), .CO(n756), .S(n834) );
  XNOR2_X1 U807 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n801) );
  OAI22_X1 U808 ( .A1(n918), .A2(n801), .B1(n769), .B2(n5), .ZN(n796) );
  XNOR2_X1 U809 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n792) );
  OAI22_X1 U810 ( .A1(n1074), .A2(n792), .B1(n1075), .B2(n770), .ZN(n795) );
  XNOR2_X1 U811 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n791) );
  OAI22_X1 U812 ( .A1(n1035), .A2(n791), .B1(n6), .B2(n771), .ZN(n794) );
  FA_X1 U813 ( .A(n774), .B(n773), .CI(n772), .CO(n804), .S(n832) );
  FA_X1 U814 ( .A(n777), .B(n776), .CI(n775), .CO(n750), .S(n782) );
  FA_X1 U815 ( .A(n780), .B(n779), .CI(n778), .CO(n755), .S(n781) );
  FA_X1 U816 ( .A(n783), .B(n782), .CI(n781), .CO(n1124), .S(n1128) );
  FA_X1 U817 ( .A(n786), .B(n785), .CI(n784), .CO(n807), .S(n837) );
  HA_X1 U818 ( .A(n788), .B(n787), .CO(n785), .S(n831) );
  XNOR2_X1 U819 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n826) );
  XNOR2_X1 U820 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[14]), 
        .ZN(n815) );
  OAI22_X1 U821 ( .A1(n1061), .A2(n815), .B1(n790), .B2(n2), .ZN(n821) );
  XNOR2_X1 U822 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n828) );
  OAI22_X1 U823 ( .A1(n1035), .A2(n828), .B1(n6), .B2(n791), .ZN(n820) );
  XNOR2_X1 U824 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n813) );
  OAI22_X1 U825 ( .A1(n1074), .A2(n813), .B1(n1075), .B2(n792), .ZN(n819) );
  XNOR2_X1 U826 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n817) );
  OAI22_X1 U827 ( .A1(n1055), .A2(n817), .B1(n793), .B2(n1057), .ZN(n818) );
  FA_X1 U828 ( .A(n796), .B(n795), .CI(n794), .CO(n833), .S(n858) );
  FA_X1 U829 ( .A(n799), .B(n798), .CI(n797), .CO(n784), .S(n857) );
  XNOR2_X1 U830 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n814) );
  OAI22_X1 U831 ( .A1(n1001), .A2(n814), .B1(n1025), .B2(n800), .ZN(n846) );
  XNOR2_X1 U832 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n825) );
  OAI22_X1 U833 ( .A1(n918), .A2(n825), .B1(n801), .B2(n5), .ZN(n845) );
  OR2_X1 U834 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n15), .ZN(n802) );
  OAI22_X1 U835 ( .A1(n802), .A2(n1088), .B1(n1090), .B2(n15), .ZN(n844) );
  FA_X1 U836 ( .A(n805), .B(n804), .CI(n803), .CO(n778), .S(n810) );
  FA_X1 U837 ( .A(n808), .B(n807), .CI(n806), .CO(n783), .S(n809) );
  FA_X1 U838 ( .A(n811), .B(n810), .CI(n809), .CO(n1127), .S(n1131) );
  INV_X1 U839 ( .A(n1088), .ZN(n812) );
  XNOR2_X1 U840 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n873) );
  XNOR2_X1 U841 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n876) );
  XNOR2_X1 U842 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n827) );
  XNOR2_X1 U843 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[12]), 
        .ZN(n881) );
  XNOR2_X1 U844 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[13]), 
        .ZN(n816) );
  OAI22_X1 U845 ( .A1(n1061), .A2(n881), .B1(n816), .B2(n2), .ZN(n878) );
  XNOR2_X1 U846 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n872) );
  OAI22_X1 U847 ( .A1(n1061), .A2(n816), .B1(n815), .B2(n2), .ZN(n848) );
  XNOR2_X1 U848 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n852) );
  OAI22_X1 U849 ( .A1(n1055), .A2(n852), .B1(n817), .B2(n1057), .ZN(n847) );
  FA_X1 U850 ( .A(n820), .B(n819), .CI(n818), .CO(n829), .S(n865) );
  HA_X1 U851 ( .A(n822), .B(n821), .CO(n830), .S(n843) );
  OAI22_X1 U852 ( .A1(n824), .A2(n1090), .B1(n1088), .B2(n823), .ZN(n842) );
  XNOR2_X1 U853 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n851) );
  OAI22_X1 U854 ( .A1(n1035), .A2(n850), .B1(n6), .B2(n828), .ZN(n853) );
  FA_X1 U855 ( .A(n831), .B(n830), .CI(n829), .CO(n836), .S(n859) );
  FA_X1 U856 ( .A(n834), .B(n833), .CI(n832), .CO(n806), .S(n839) );
  FA_X1 U857 ( .A(n837), .B(n836), .CI(n835), .CO(n811), .S(n838) );
  FA_X1 U858 ( .A(n840), .B(n839), .CI(n838), .CO(n1130), .S(n1134) );
  FA_X1 U859 ( .A(n843), .B(n842), .CI(n841), .CO(n860), .S(n885) );
  FA_X1 U860 ( .A(n846), .B(n845), .CI(n844), .CO(n856), .S(n884) );
  FA_X1 U861 ( .A(n849), .B(n848), .CI(n847), .CO(n866), .S(n903) );
  XNOR2_X1 U862 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n877) );
  XNOR2_X1 U863 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n875) );
  XNOR2_X1 U864 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n882) );
  OAI22_X1 U865 ( .A1(n1055), .A2(n882), .B1(n852), .B2(n1057), .ZN(n895) );
  FA_X1 U866 ( .A(n855), .B(n854), .CI(n853), .CO(n841), .S(n901) );
  FA_X1 U867 ( .A(n858), .B(n857), .CI(n856), .CO(n835), .S(n863) );
  FA_X1 U868 ( .A(n861), .B(n860), .CI(n859), .CO(n840), .S(n862) );
  FA_X1 U869 ( .A(n864), .B(n863), .CI(n862), .CO(n1133), .S(n1137) );
  FA_X1 U870 ( .A(n867), .B(n866), .CI(n865), .CO(n861), .S(n888) );
  FA_X1 U871 ( .A(n870), .B(n869), .CI(n868), .CO(n867), .S(n906) );
  OR2_X1 U872 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n16), .ZN(n871) );
  XNOR2_X1 U873 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n880) );
  XNOR2_X1 U874 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[13]), 
        .ZN(n874) );
  XNOR2_X1 U875 ( .A(xbuf_imag_stage2[11]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n916) );
  XNOR2_X1 U876 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n889) );
  XNOR2_X1 U877 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n892) );
  HA_X1 U878 ( .A(n879), .B(n878), .CO(n868), .S(n923) );
  XNOR2_X1 U879 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n893) );
  XNOR2_X1 U880 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[11]), 
        .ZN(n890) );
  OAI22_X1 U881 ( .A1(n1061), .A2(n890), .B1(n881), .B2(n2), .ZN(n914) );
  XNOR2_X1 U882 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n894) );
  FA_X1 U883 ( .A(n885), .B(n884), .CI(n883), .CO(n864), .S(n886) );
  FA_X1 U884 ( .A(n888), .B(n887), .CI(n886), .CO(n1136), .S(n1140) );
  XNOR2_X1 U885 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n935) );
  XNOR2_X1 U886 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[10]), 
        .ZN(n939) );
  OAI22_X1 U887 ( .A1(n1061), .A2(n939), .B1(n890), .B2(n2), .ZN(n920) );
  INV_X1 U888 ( .A(n1075), .ZN(n891) );
  XNOR2_X1 U889 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n934) );
  XNOR2_X1 U890 ( .A(xbuf_imag_stage2[7]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n938) );
  FA_X1 U891 ( .A(n897), .B(n896), .CI(n895), .CO(n902), .S(n926) );
  FA_X1 U892 ( .A(n900), .B(n899), .CI(n898), .CO(n905), .S(n925) );
  FA_X1 U893 ( .A(n903), .B(n902), .CI(n901), .CO(n883), .S(n908) );
  FA_X1 U894 ( .A(n906), .B(n905), .CI(n904), .CO(n887), .S(n907) );
  FA_X1 U895 ( .A(n909), .B(n908), .CI(n907), .CO(n1139), .S(n1143) );
  FA_X1 U896 ( .A(n912), .B(n911), .CI(n910), .CO(n924), .S(n946) );
  FA_X1 U897 ( .A(n915), .B(n914), .CI(n913), .CO(n922), .S(n945) );
  XNOR2_X1 U898 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[11]), 
        .ZN(n917) );
  OR2_X1 U899 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n17), .ZN(n919) );
  HA_X1 U900 ( .A(n921), .B(n920), .CO(n933), .S(n958) );
  FA_X1 U901 ( .A(n924), .B(n923), .CI(n922), .CO(n904), .S(n929) );
  FA_X1 U902 ( .A(n927), .B(n926), .CI(n925), .CO(n909), .S(n928) );
  FA_X1 U903 ( .A(n930), .B(n929), .CI(n928), .CO(n1142), .S(n1146) );
  FA_X1 U904 ( .A(n933), .B(n932), .CI(n931), .CO(n927), .S(n949) );
  XNOR2_X1 U905 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n956) );
  XNOR2_X1 U906 ( .A(xbuf_imag_stage2[9]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n969) );
  AND2_X1 U907 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n937), .ZN(n975) );
  XNOR2_X1 U908 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[9]), 
        .ZN(n954) );
  OAI22_X1 U909 ( .A1(n1061), .A2(n954), .B1(n939), .B2(n2), .ZN(n951) );
  XNOR2_X1 U910 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n955) );
  FA_X1 U911 ( .A(n943), .B(n942), .CI(n941), .CO(n931), .S(n961) );
  FA_X1 U912 ( .A(n946), .B(n945), .CI(n944), .CO(n930), .S(n947) );
  FA_X1 U913 ( .A(n949), .B(n948), .CI(n947), .CO(n1145), .S(n1149) );
  FA_X1 U914 ( .A(n952), .B(n951), .CI(n950), .CO(n962), .S(n980) );
  XNOR2_X1 U915 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[8]), 
        .ZN(n973) );
  XNOR2_X1 U916 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n974) );
  XNOR2_X1 U917 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n989) );
  OR2_X1 U918 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n18), .ZN(n957) );
  FA_X1 U919 ( .A(n960), .B(n959), .CI(n958), .CO(n944), .S(n965) );
  FA_X1 U920 ( .A(n963), .B(n962), .CI(n961), .CO(n948), .S(n964) );
  FA_X1 U921 ( .A(n966), .B(n965), .CI(n964), .CO(n1148), .S(n1152) );
  HA_X1 U922 ( .A(n968), .B(n967), .CO(n979), .S(n994) );
  XNOR2_X1 U923 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[9]), 
        .ZN(n971) );
  XNOR2_X1 U924 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[7]), 
        .ZN(n991) );
  OAI22_X1 U925 ( .A1(n1061), .A2(n991), .B1(n973), .B2(n2), .ZN(n1004) );
  XNOR2_X1 U926 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n999) );
  FA_X1 U927 ( .A(n977), .B(n976), .CI(n975), .CO(n963), .S(n982) );
  FA_X1 U928 ( .A(n980), .B(n979), .CI(n978), .CO(n966), .S(n981) );
  FA_X1 U929 ( .A(n983), .B(n982), .CI(n981), .CO(n1151), .S(n1155) );
  FA_X1 U930 ( .A(n986), .B(n985), .CI(n984), .CO(n978), .S(n997) );
  INV_X1 U931 ( .A(n987), .ZN(n988) );
  AND2_X1 U932 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n988), .ZN(n1008) );
  XNOR2_X1 U933 ( .A(xbuf_imag_stage2[5]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n990) );
  XNOR2_X1 U934 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[6]), 
        .ZN(n1013) );
  FA_X1 U935 ( .A(n994), .B(n993), .CI(n992), .CO(n983), .S(n995) );
  FA_X1 U936 ( .A(n997), .B(n996), .CI(n995), .CO(n1154), .S(n1158) );
  OR2_X1 U937 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n19), .ZN(n998) );
  XNOR2_X1 U938 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n1014) );
  XNOR2_X1 U939 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[7]), 
        .ZN(n1002) );
  FA_X1 U940 ( .A(n1005), .B(n1004), .CI(n1003), .CO(n992), .S(n1010) );
  FA_X1 U941 ( .A(n1008), .B(n1007), .CI(n1006), .CO(n996), .S(n1009) );
  FA_X1 U942 ( .A(n1011), .B(n1010), .CI(n1009), .CO(n1157), .S(n1161) );
  XNOR2_X1 U943 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[5]), 
        .ZN(n1024) );
  OAI22_X1 U944 ( .A1(n1061), .A2(n1024), .B1(n1013), .B2(n2), .ZN(n1028) );
  XNOR2_X1 U945 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n1023) );
  HA_X1 U946 ( .A(n1016), .B(n1015), .CO(n1006), .S(n1021) );
  FA_X1 U947 ( .A(n1019), .B(n1018), .CI(n1017), .CO(n1011), .S(n1020) );
  FA_X1 U948 ( .A(n1022), .B(n1021), .CI(n1020), .CO(n1160), .S(n1164) );
  XNOR2_X1 U949 ( .A(xbuf_imag_stage2[3]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n1042) );
  XNOR2_X1 U950 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[4]), 
        .ZN(n1043) );
  OAI22_X1 U951 ( .A1(n1061), .A2(n1043), .B1(n1024), .B2(n2), .ZN(n1037) );
  INV_X1 U952 ( .A(n1025), .ZN(n1026) );
  AND2_X1 U953 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n1026), .ZN(n1031) );
  FA_X1 U954 ( .A(n1029), .B(n1028), .CI(n1027), .CO(n1022), .S(n1030) );
  FA_X1 U955 ( .A(n1032), .B(n1031), .CI(n1030), .CO(n1163), .S(n1167) );
  XNOR2_X1 U956 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[5]), 
        .ZN(n1034) );
  OAI22_X1 U957 ( .A1(n1034), .A2(n1035), .B1(n6), .B2(n1033), .ZN(n1041) );
  OR2_X1 U958 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n20), .ZN(n1036) );
  OAI22_X1 U959 ( .A1(n1036), .A2(n6), .B1(n1035), .B2(n20), .ZN(n1040) );
  HA_X1 U960 ( .A(n1038), .B(n1037), .CO(n1032), .S(n1039) );
  FA_X1 U961 ( .A(n1041), .B(n1040), .CI(n1039), .CO(n1166), .S(n1170) );
  OAI22_X1 U962 ( .A1(n1055), .A2(n1054), .B1(n1042), .B2(n1057), .ZN(n1048)
         );
  XNOR2_X1 U963 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[3]), 
        .ZN(n1051) );
  OAI22_X1 U964 ( .A1(n1061), .A2(n1051), .B1(n1043), .B2(n2), .ZN(n1047) );
  INV_X1 U965 ( .A(n1044), .ZN(n1045) );
  AND2_X1 U966 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n1045), .ZN(n1046) );
  FA_X1 U967 ( .A(n1048), .B(n1047), .CI(n1046), .CO(n1169), .S(n1173) );
  OR2_X1 U968 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n21), .ZN(n1049) );
  OR2_X1 U969 ( .A1(n1057), .A2(n1049), .ZN(n1050) );
  OAI21_X1 U970 ( .B1(n21), .B2(n1055), .A(n1050), .ZN(n1053) );
  XNOR2_X1 U971 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[2]), 
        .ZN(n1058) );
  OAI22_X1 U972 ( .A1(n1061), .A2(n1058), .B1(n1051), .B2(n2), .ZN(n1052) );
  HA_X1 U973 ( .A(n1053), .B(n1052), .CO(n1172), .S(n1176) );
  XNOR2_X1 U974 ( .A(twiddle_real_m_imag_stage2[0]), .B(xbuf_imag_stage2[3]), 
        .ZN(n1056) );
  OAI22_X1 U975 ( .A1(n1056), .A2(n1055), .B1(n1054), .B2(n1057), .ZN(n1175)
         );
  AND2_X1 U976 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n674), .ZN(n1179) );
  XNOR2_X1 U977 ( .A(xbuf_imag_stage2[1]), .B(twiddle_real_p_imag_stage2[1]), 
        .ZN(n1059) );
  OAI22_X1 U978 ( .A1(n1061), .A2(n1059), .B1(n1058), .B2(n2), .ZN(n1178) );
  OAI22_X1 U979 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n1061), .B1(n1059), 
        .B2(n2), .ZN(n1181) );
  OR2_X1 U980 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(n38), .ZN(n1060) );
  NAND2_X1 U981 ( .A1(n1061), .A2(n1060), .ZN(n1180) );
  NAND2_X1 U982 ( .A1(n1181), .A2(n1180), .ZN(n1182) );
  INV_X1 U983 ( .A(n1182), .ZN(n1177) );
  FA_X1 U984 ( .A(n1063), .B(n27), .CI(n1062), .CO(n1081), .S(n1101) );
  XNOR2_X1 U985 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[15]), 
        .ZN(n1073) );
  OAI22_X1 U986 ( .A1(n1090), .A2(n1064), .B1(n1088), .B2(n1073), .ZN(n1085)
         );
  INV_X1 U987 ( .A(n1085), .ZN(n1072) );
  XNOR2_X1 U988 ( .A(xbuf_imag_stage2[13]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n1076) );
  FA_X1 U989 ( .A(n1068), .B(n1067), .CI(n1066), .CO(n1070), .S(n1062) );
  FA_X1 U990 ( .A(n1072), .B(n1071), .CI(n1070), .CO(n1098), .S(n1082) );
  XNOR2_X1 U991 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[16]), 
        .ZN(n1089) );
  OAI22_X1 U992 ( .A1(n1090), .A2(n1073), .B1(n1088), .B2(n1089), .ZN(n1086)
         );
  OR2_X1 U993 ( .A1(n1076), .A2(n1074), .ZN(n1078) );
  OR2_X1 U994 ( .A1(n1076), .A2(n1075), .ZN(n1077) );
  NAND2_X1 U995 ( .A1(n1080), .A2(n1079), .ZN(n1084) );
  NAND2_X1 U996 ( .A1(n1082), .A2(n1081), .ZN(n1083) );
  NAND2_X1 U997 ( .A1(n1084), .A2(n1083), .ZN(n1096) );
  FA_X1 U998 ( .A(n1086), .B(n1085), .CI(n23), .CO(n1093), .S(n1097) );
  XNOR2_X1 U999 ( .A(xbuf_imag_stage2[15]), .B(twiddle_real_p_imag_stage2[17]), 
        .ZN(n1087) );
  OAI22_X1 U1000 ( .A1(n1090), .A2(n1089), .B1(n1088), .B2(n1087), .ZN(n1091)
         );
  INV_X1 U1001 ( .A(n1091), .ZN(n1092) );
  XOR2_X1 U1002 ( .A(n1095), .B(n1094), .Z(tmp_r[31]) );
  FA_X1 U1003 ( .A(n1098), .B(n1097), .CI(n1096), .CO(n1095), .S(tmp_r[30]) );
  FA_X1 U1004 ( .A(n1101), .B(n1100), .CI(n1099), .CO(n1080), .S(tmp_r[28]) );
  FA_X1 U1005 ( .A(n1104), .B(n1103), .CI(n1102), .CO(n1099), .S(tmp_r[27]) );
  FA_X1 U1006 ( .A(n1107), .B(n1106), .CI(n1105), .CO(n1102), .S(tmp_r[26]) );
  FA_X1 U1007 ( .A(n1110), .B(n1109), .CI(n1108), .CO(n1105), .S(tmp_r[25]) );
  FA_X1 U1008 ( .A(n1113), .B(n1112), .CI(n1111), .CO(n1108), .S(tmp_r[24]) );
  FA_X1 U1009 ( .A(n1116), .B(n1115), .CI(n1114), .CO(n1111), .S(tmp_r[23]) );
  FA_X1 U1010 ( .A(n1119), .B(n1118), .CI(n1117), .CO(n1114), .S(tmp_r[22]) );
  FA_X1 U1011 ( .A(n1122), .B(n1121), .CI(n1120), .CO(n1117), .S(tmp_r[21]) );
  FA_X1 U1012 ( .A(n1125), .B(n1124), .CI(n1123), .CO(n1120), .S(tmp_r[20]) );
  FA_X1 U1013 ( .A(n1128), .B(n1127), .CI(n1126), .CO(n1123), .S(tmp_r[19]) );
  FA_X1 U1014 ( .A(n1131), .B(n1130), .CI(n1129), .CO(n1126), .S(tmp_r[18]) );
  FA_X1 U1015 ( .A(n1134), .B(n1133), .CI(n1132), .CO(n1129), .S(tmp_r[17]) );
  FA_X1 U1016 ( .A(n1137), .B(n1136), .CI(n1135), .CO(n1132), .S(tmp_r[16]) );
  FA_X1 U1017 ( .A(n1140), .B(n1139), .CI(n1138), .CO(n1135), .S(tmp_r[15]) );
  FA_X1 U1018 ( .A(n1143), .B(n1142), .CI(n1141), .CO(n1138), .S(tmp_r[14]) );
  FA_X1 U1019 ( .A(n1146), .B(n1145), .CI(n1144), .CO(n1141), .S(tmp_r[13]) );
  FA_X1 U1020 ( .A(n1149), .B(n1148), .CI(n1147), .CO(n1144), .S(tmp_r[12]) );
  FA_X1 U1021 ( .A(n1152), .B(n1151), .CI(n1150), .CO(n1147), .S(tmp_r[11]) );
  FA_X1 U1022 ( .A(n1155), .B(n1154), .CI(n1153), .CO(n1150), .S(tmp_r[10]) );
  FA_X1 U1023 ( .A(n1158), .B(n1157), .CI(n1156), .CO(n1153), .S(tmp_r[9]) );
  FA_X1 U1024 ( .A(n1161), .B(n1160), .CI(n1159), .CO(n1156), .S(tmp_r[8]) );
  FA_X1 U1025 ( .A(n1164), .B(n1163), .CI(n1162), .CO(n1159), .S(tmp_r[7]) );
  FA_X1 U1026 ( .A(n1167), .B(n1166), .CI(n1165), .CO(n1162), .S(tmp_r[6]) );
  FA_X1 U1027 ( .A(n1170), .B(n1169), .CI(n1168), .CO(n1165), .S(tmp_r[5]) );
  FA_X1 U1028 ( .A(n1173), .B(n1172), .CI(n1171), .CO(n1168), .S(tmp_r[4]) );
  FA_X1 U1029 ( .A(n1176), .B(n1175), .CI(n1174), .CO(n1171), .S(tmp_r[3]) );
  FA_X1 U1030 ( .A(n1179), .B(n1178), .CI(n1177), .CO(n1174), .S(tmp_r[2]) );
  AND2_X1 U1031 ( .A1(twiddle_real_m_imag_stage2[0]), .A2(xbuf_imag_stage2[0]), 
        .ZN(tmp_r[0]) );
  FA_X1 U1032 ( .A(n1185), .B(n1184), .CI(n1183), .CO(n568), .S(tmp_i[30]) );
  FA_X1 U1033 ( .A(n1188), .B(n1187), .CI(n1186), .CO(n553), .S(tmp_i[28]) );
  FA_X1 U1034 ( .A(n1191), .B(n1190), .CI(n1189), .CO(n1186), .S(tmp_i[27]) );
  FA_X1 U1035 ( .A(n1194), .B(n1193), .CI(n1192), .CO(n1189), .S(tmp_i[26]) );
  FA_X1 U1036 ( .A(n1197), .B(n1196), .CI(n1195), .CO(n1192), .S(tmp_i[25]) );
  FA_X1 U1037 ( .A(n1200), .B(n1199), .CI(n1198), .CO(n1195), .S(tmp_i[24]) );
  FA_X1 U1038 ( .A(n1203), .B(n1202), .CI(n1201), .CO(n1198), .S(tmp_i[23]) );
  FA_X1 U1039 ( .A(n1206), .B(n1205), .CI(n1204), .CO(n1201), .S(tmp_i[22]) );
  FA_X1 U1040 ( .A(n1209), .B(n1208), .CI(n1207), .CO(n1204), .S(tmp_i[21]) );
  FA_X1 U1041 ( .A(n1212), .B(n1211), .CI(n1210), .CO(n1207), .S(tmp_i[20]) );
  FA_X1 U1042 ( .A(n1215), .B(n1214), .CI(n1213), .CO(n1210), .S(tmp_i[19]) );
  FA_X1 U1043 ( .A(n1218), .B(n1217), .CI(n1216), .CO(n1213), .S(tmp_i[18]) );
  FA_X1 U1044 ( .A(n1221), .B(n1220), .CI(n1219), .CO(n1216), .S(tmp_i[17]) );
  FA_X1 U1045 ( .A(n1224), .B(n1223), .CI(n1222), .CO(n1219), .S(tmp_i[16]) );
  FA_X1 U1046 ( .A(n1227), .B(n1226), .CI(n1225), .CO(n1222), .S(tmp_i[15]) );
  FA_X1 U1047 ( .A(n1230), .B(n1229), .CI(n1228), .CO(n1225), .S(tmp_i[14]) );
  FA_X1 U1048 ( .A(n1233), .B(n1232), .CI(n1231), .CO(n1228), .S(tmp_i[13]) );
  FA_X1 U1049 ( .A(n1236), .B(n1235), .CI(n1234), .CO(n1231), .S(tmp_i[12]) );
  FA_X1 U1050 ( .A(n1239), .B(n1238), .CI(n1237), .CO(n1234), .S(tmp_i[11]) );
  FA_X1 U1051 ( .A(n1242), .B(n1241), .CI(n1240), .CO(n1237), .S(tmp_i[10]) );
  FA_X1 U1052 ( .A(n1245), .B(n1244), .CI(n1243), .CO(n1240), .S(tmp_i[9]) );
  FA_X1 U1053 ( .A(n1248), .B(n1247), .CI(n1246), .CO(n1243), .S(tmp_i[8]) );
  FA_X1 U1054 ( .A(n1251), .B(n1250), .CI(n1249), .CO(n1246), .S(tmp_i[7]) );
  FA_X1 U1055 ( .A(n1254), .B(n1253), .CI(n1252), .CO(n1249), .S(tmp_i[6]) );
  FA_X1 U1056 ( .A(n1257), .B(n1256), .CI(n1255), .CO(n1252), .S(tmp_i[5]) );
  FA_X1 U1057 ( .A(n1260), .B(n1259), .CI(n1258), .CO(n1255), .S(tmp_i[4]) );
  FA_X1 U1058 ( .A(n1263), .B(n1262), .CI(n1261), .CO(n1258), .S(tmp_i[3]) );
  FA_X1 U1059 ( .A(n1266), .B(n1265), .CI(n1264), .CO(n1261), .S(tmp_i[2]) );
  INV_X1 U1060 ( .A(opb_imag[0]), .ZN(n1270) );
  NOR2_X1 U1061 ( .A1(opa_imag[0]), .A2(n1270), .ZN(n1276) );
  INV_X1 U1062 ( .A(opb_imag[1]), .ZN(n1271) );
  NOR2_X1 U1063 ( .A1(opa_imag[1]), .A2(n1271), .ZN(n1275) );
  AOI21_X1 U1064 ( .B1(opa_imag[1]), .B2(n1271), .A(n1275), .ZN(n1895) );
  XOR2_X1 U1065 ( .A(n1276), .B(n1895), .Z(n1957) );
  INV_X1 U1066 ( .A(n1957), .ZN(xbuf_imag_stage1[0]) );
  INV_X1 U1067 ( .A(opb_real[0]), .ZN(n1272) );
  NOR2_X1 U1068 ( .A1(opa_real[0]), .A2(n1272), .ZN(n1288) );
  INV_X1 U1069 ( .A(opb_real[1]), .ZN(n1273) );
  NOR2_X1 U1070 ( .A1(opa_real[1]), .A2(n1273), .ZN(n1287) );
  AOI21_X1 U1071 ( .B1(opa_real[1]), .B2(n1273), .A(n1287), .ZN(n1848) );
  XOR2_X1 U1072 ( .A(n1288), .B(n1848), .Z(n1956) );
  INV_X1 U1073 ( .A(n1956), .ZN(xbuf_real_stage1[0]) );
  INV_X1 U1074 ( .A(\intadd_0/SUM[0] ), .ZN(tmp_a[4]) );
  INV_X1 U1075 ( .A(\intadd_0/SUM[1] ), .ZN(tmp_a[5]) );
  INV_X1 U1076 ( .A(\intadd_0/SUM[2] ), .ZN(tmp_a[6]) );
  INV_X1 U1077 ( .A(\intadd_0/SUM[3] ), .ZN(tmp_a[7]) );
  INV_X1 U1078 ( .A(\intadd_0/SUM[4] ), .ZN(tmp_a[8]) );
  INV_X1 U1079 ( .A(opa_imag[15]), .ZN(n1899) );
  NOR2_X1 U1080 ( .A1(opb_imag[15]), .A2(n1899), .ZN(n1285) );
  INV_X1 U1081 ( .A(opb_imag[14]), .ZN(n1858) );
  INV_X1 U1082 ( .A(opa_imag[14]), .ZN(n1859) );
  NAND2_X1 U1083 ( .A1(opb_imag[14]), .A2(n1859), .ZN(n1284) );
  INV_X1 U1084 ( .A(opa_imag[13]), .ZN(n1861) );
  AND2_X1 U1085 ( .A1(n1861), .A2(opb_imag[13]), .ZN(n1283) );
  INV_X1 U1086 ( .A(opb_imag[12]), .ZN(n1863) );
  INV_X1 U1087 ( .A(opa_imag[12]), .ZN(n1864) );
  NAND2_X1 U1088 ( .A1(opb_imag[12]), .A2(n1864), .ZN(n1282) );
  INV_X1 U1089 ( .A(opa_imag[11]), .ZN(n1866) );
  AND2_X1 U1090 ( .A1(n1866), .A2(opb_imag[11]), .ZN(n1281) );
  INV_X1 U1091 ( .A(opb_imag[10]), .ZN(n1857) );
  INV_X1 U1092 ( .A(opa_imag[10]), .ZN(n1869) );
  NAND2_X1 U1093 ( .A1(opb_imag[10]), .A2(n1869), .ZN(n1868) );
  INV_X1 U1094 ( .A(opb_imag[9]), .ZN(n1872) );
  NOR2_X1 U1095 ( .A1(opa_imag[9]), .A2(n1872), .ZN(n1871) );
  INV_X1 U1096 ( .A(opb_imag[8]), .ZN(n1856) );
  INV_X1 U1097 ( .A(opa_imag[8]), .ZN(n1875) );
  NAND2_X1 U1098 ( .A1(opb_imag[8]), .A2(n1875), .ZN(n1874) );
  INV_X1 U1099 ( .A(opb_imag[7]), .ZN(n1878) );
  NOR2_X1 U1100 ( .A1(opa_imag[7]), .A2(n1878), .ZN(n1877) );
  INV_X1 U1101 ( .A(opb_imag[6]), .ZN(n1855) );
  INV_X1 U1102 ( .A(opa_imag[6]), .ZN(n1881) );
  NAND2_X1 U1103 ( .A1(opb_imag[6]), .A2(n1881), .ZN(n1880) );
  INV_X1 U1104 ( .A(opb_imag[5]), .ZN(n1884) );
  NOR2_X1 U1105 ( .A1(opa_imag[5]), .A2(n1884), .ZN(n1883) );
  INV_X1 U1106 ( .A(opb_imag[4]), .ZN(n1854) );
  INV_X1 U1107 ( .A(opa_imag[4]), .ZN(n1887) );
  NAND2_X1 U1108 ( .A1(opb_imag[4]), .A2(n1887), .ZN(n1886) );
  INV_X1 U1109 ( .A(opb_imag[3]), .ZN(n1890) );
  NOR2_X1 U1110 ( .A1(opa_imag[3]), .A2(n1890), .ZN(n1889) );
  INV_X1 U1111 ( .A(opb_imag[2]), .ZN(n1853) );
  INV_X1 U1112 ( .A(opa_imag[2]), .ZN(n1893) );
  NAND2_X1 U1113 ( .A1(opb_imag[2]), .A2(n1893), .ZN(n1892) );
  INV_X1 U1114 ( .A(opa_imag[1]), .ZN(n1274) );
  OAI22_X1 U1115 ( .A1(n1276), .A2(n1275), .B1(opb_imag[1]), .B2(n1274), .ZN(
        n1929) );
  AOI22_X1 U1116 ( .A1(opa_imag[2]), .A2(n1853), .B1(n1892), .B2(n1929), .ZN(
        n1925) );
  INV_X1 U1117 ( .A(opa_imag[3]), .ZN(n1277) );
  OAI22_X1 U1118 ( .A1(n1889), .A2(n1925), .B1(opb_imag[3]), .B2(n1277), .ZN(
        n1923) );
  AOI22_X1 U1119 ( .A1(opa_imag[4]), .A2(n1854), .B1(n1886), .B2(n1923), .ZN(
        n1921) );
  INV_X1 U1120 ( .A(opa_imag[5]), .ZN(n1278) );
  OAI22_X1 U1121 ( .A1(n1883), .A2(n1921), .B1(opb_imag[5]), .B2(n1278), .ZN(
        n1919) );
  AOI22_X1 U1122 ( .A1(opa_imag[6]), .A2(n1855), .B1(n1880), .B2(n1919), .ZN(
        n1917) );
  INV_X1 U1123 ( .A(opa_imag[7]), .ZN(n1279) );
  OAI22_X1 U1124 ( .A1(n1877), .A2(n1917), .B1(opb_imag[7]), .B2(n1279), .ZN(
        n1915) );
  AOI22_X1 U1125 ( .A1(opa_imag[8]), .A2(n1856), .B1(n1874), .B2(n1915), .ZN(
        n1913) );
  INV_X1 U1126 ( .A(opa_imag[9]), .ZN(n1280) );
  OAI22_X1 U1127 ( .A1(n1871), .A2(n1913), .B1(opb_imag[9]), .B2(n1280), .ZN(
        n1911) );
  AOI22_X1 U1128 ( .A1(opa_imag[10]), .A2(n1857), .B1(n1868), .B2(n1911), .ZN(
        n1908) );
  OAI22_X1 U1129 ( .A1(n1281), .A2(n1908), .B1(opb_imag[11]), .B2(n1866), .ZN(
        n1906) );
  AOI22_X1 U1130 ( .A1(opa_imag[12]), .A2(n1863), .B1(n1282), .B2(n1906), .ZN(
        n1904) );
  OAI22_X1 U1131 ( .A1(n1283), .A2(n1904), .B1(opb_imag[13]), .B2(n1861), .ZN(
        n1902) );
  AOI22_X1 U1132 ( .A1(opa_imag[14]), .A2(n1858), .B1(n1284), .B2(n1902), .ZN(
        n1900) );
  INV_X1 U1133 ( .A(opb_imag[15]), .ZN(n1898) );
  OAI22_X1 U1134 ( .A1(n1285), .A2(n1900), .B1(opa_imag[15]), .B2(n1898), .ZN(
        \intadd_3/B[14] ) );
  INV_X1 U1135 ( .A(\intadd_3/B[14] ), .ZN(xbuf_imag_stage1[15]) );
  INV_X1 U1136 ( .A(\intadd_0/SUM[5] ), .ZN(tmp_a[9]) );
  INV_X1 U1137 ( .A(opa_real[15]), .ZN(n1852) );
  NOR2_X1 U1138 ( .A1(opb_real[15]), .A2(n1852), .ZN(n1297) );
  INV_X1 U1139 ( .A(opb_real[14]), .ZN(n1811) );
  INV_X1 U1140 ( .A(opa_real[14]), .ZN(n1812) );
  NAND2_X1 U1141 ( .A1(opb_real[14]), .A2(n1812), .ZN(n1296) );
  INV_X1 U1142 ( .A(opa_real[13]), .ZN(n1814) );
  AND2_X1 U1143 ( .A1(n1814), .A2(opb_real[13]), .ZN(n1295) );
  INV_X1 U1144 ( .A(opb_real[12]), .ZN(n1816) );
  INV_X1 U1145 ( .A(opa_real[12]), .ZN(n1817) );
  NAND2_X1 U1146 ( .A1(opb_real[12]), .A2(n1817), .ZN(n1294) );
  INV_X1 U1147 ( .A(opa_real[11]), .ZN(n1819) );
  AND2_X1 U1148 ( .A1(n1819), .A2(opb_real[11]), .ZN(n1293) );
  INV_X1 U1149 ( .A(opb_real[10]), .ZN(n1810) );
  INV_X1 U1150 ( .A(opa_real[10]), .ZN(n1822) );
  NAND2_X1 U1151 ( .A1(opb_real[10]), .A2(n1822), .ZN(n1821) );
  INV_X1 U1152 ( .A(opb_real[9]), .ZN(n1825) );
  NOR2_X1 U1153 ( .A1(opa_real[9]), .A2(n1825), .ZN(n1824) );
  INV_X1 U1154 ( .A(opb_real[8]), .ZN(n1809) );
  INV_X1 U1155 ( .A(opa_real[8]), .ZN(n1828) );
  NAND2_X1 U1156 ( .A1(opb_real[8]), .A2(n1828), .ZN(n1827) );
  INV_X1 U1157 ( .A(opb_real[7]), .ZN(n1831) );
  NOR2_X1 U1158 ( .A1(opa_real[7]), .A2(n1831), .ZN(n1830) );
  INV_X1 U1159 ( .A(opb_real[6]), .ZN(n1808) );
  INV_X1 U1160 ( .A(opa_real[6]), .ZN(n1834) );
  NAND2_X1 U1161 ( .A1(opb_real[6]), .A2(n1834), .ZN(n1833) );
  INV_X1 U1162 ( .A(opb_real[5]), .ZN(n1837) );
  NOR2_X1 U1163 ( .A1(opa_real[5]), .A2(n1837), .ZN(n1836) );
  INV_X1 U1164 ( .A(opb_real[4]), .ZN(n1807) );
  INV_X1 U1165 ( .A(opa_real[4]), .ZN(n1840) );
  NAND2_X1 U1166 ( .A1(opb_real[4]), .A2(n1840), .ZN(n1839) );
  INV_X1 U1167 ( .A(opb_real[3]), .ZN(n1843) );
  NOR2_X1 U1168 ( .A1(opa_real[3]), .A2(n1843), .ZN(n1842) );
  INV_X1 U1169 ( .A(opb_real[2]), .ZN(n1806) );
  INV_X1 U1170 ( .A(opa_real[2]), .ZN(n1846) );
  NAND2_X1 U1171 ( .A1(opb_real[2]), .A2(n1846), .ZN(n1845) );
  INV_X1 U1172 ( .A(opa_real[1]), .ZN(n1286) );
  OAI22_X1 U1173 ( .A1(n1288), .A2(n1287), .B1(opb_real[1]), .B2(n1286), .ZN(
        n1927) );
  AOI22_X1 U1174 ( .A1(opa_real[2]), .A2(n1806), .B1(n1845), .B2(n1927), .ZN(
        n1931) );
  INV_X1 U1175 ( .A(opa_real[3]), .ZN(n1289) );
  OAI22_X1 U1176 ( .A1(n1842), .A2(n1931), .B1(opb_real[3]), .B2(n1289), .ZN(
        n1933) );
  AOI22_X1 U1177 ( .A1(opa_real[4]), .A2(n1807), .B1(n1839), .B2(n1933), .ZN(
        n1935) );
  INV_X1 U1178 ( .A(opa_real[5]), .ZN(n1290) );
  OAI22_X1 U1179 ( .A1(n1836), .A2(n1935), .B1(opb_real[5]), .B2(n1290), .ZN(
        n1937) );
  AOI22_X1 U1180 ( .A1(opa_real[6]), .A2(n1808), .B1(n1833), .B2(n1937), .ZN(
        n1939) );
  INV_X1 U1181 ( .A(opa_real[7]), .ZN(n1291) );
  OAI22_X1 U1182 ( .A1(n1830), .A2(n1939), .B1(opb_real[7]), .B2(n1291), .ZN(
        n1941) );
  AOI22_X1 U1183 ( .A1(opa_real[8]), .A2(n1809), .B1(n1827), .B2(n1941), .ZN(
        n1943) );
  INV_X1 U1184 ( .A(opa_real[9]), .ZN(n1292) );
  OAI22_X1 U1185 ( .A1(n1824), .A2(n1943), .B1(opb_real[9]), .B2(n1292), .ZN(
        n1945) );
  AOI22_X1 U1186 ( .A1(opa_real[10]), .A2(n1810), .B1(n1821), .B2(n1945), .ZN(
        n1946) );
  OAI22_X1 U1187 ( .A1(n1293), .A2(n1946), .B1(opb_real[11]), .B2(n1819), .ZN(
        n1948) );
  AOI22_X1 U1188 ( .A1(opa_real[12]), .A2(n1816), .B1(n1294), .B2(n1948), .ZN(
        n1950) );
  OAI22_X1 U1189 ( .A1(n1295), .A2(n1950), .B1(opb_real[13]), .B2(n1814), .ZN(
        n1952) );
  AOI22_X1 U1190 ( .A1(opa_real[14]), .A2(n1811), .B1(n1296), .B2(n1952), .ZN(
        n1954) );
  INV_X1 U1191 ( .A(opb_real[15]), .ZN(n1851) );
  OAI22_X1 U1192 ( .A1(n1297), .A2(n1954), .B1(opa_real[15]), .B2(n1851), .ZN(
        \intadd_3/A[14] ) );
  INV_X1 U1193 ( .A(\intadd_3/A[14] ), .ZN(xbuf_real_stage1[15]) );
  INV_X1 U1194 ( .A(\intadd_0/SUM[6] ), .ZN(tmp_a[10]) );
  INV_X1 U1195 ( .A(\intadd_0/SUM[7] ), .ZN(tmp_a[11]) );
  INV_X1 U1196 ( .A(\intadd_0/SUM[8] ), .ZN(tmp_a[12]) );
  INV_X1 U1197 ( .A(\intadd_0/SUM[9] ), .ZN(tmp_a[13]) );
  INV_X1 U1198 ( .A(\intadd_0/SUM[10] ), .ZN(tmp_a[14]) );
  INV_X1 U1199 ( .A(\intadd_0/SUM[11] ), .ZN(tmp_a[15]) );
  INV_X1 U1200 ( .A(\intadd_3/n1 ), .ZN(xbuf_real_p_imag_stage1[16]) );
  INV_X1 U1201 ( .A(\intadd_0/SUM[12] ), .ZN(tmp_a[16]) );
  INV_X1 U1202 ( .A(\intadd_0/SUM[13] ), .ZN(tmp_a[17]) );
  INV_X1 U1203 ( .A(\intadd_0/SUM[14] ), .ZN(tmp_a[18]) );
  INV_X1 U1204 ( .A(\intadd_0/SUM[15] ), .ZN(tmp_a[19]) );
  INV_X1 U1205 ( .A(\intadd_0/SUM[16] ), .ZN(tmp_a[20]) );
  INV_X1 U1206 ( .A(\intadd_0/SUM[17] ), .ZN(tmp_a[21]) );
  INV_X1 U1207 ( .A(\intadd_0/SUM[18] ), .ZN(tmp_a[22]) );
  INV_X1 U1208 ( .A(\intadd_0/SUM[19] ), .ZN(tmp_a[23]) );
  INV_X1 U1209 ( .A(\intadd_0/SUM[20] ), .ZN(tmp_a[24]) );
  INV_X1 U1210 ( .A(\intadd_0/SUM[21] ), .ZN(tmp_a[25]) );
  INV_X1 U1211 ( .A(\intadd_0/SUM[22] ), .ZN(tmp_a[26]) );
  INV_X1 U1212 ( .A(\intadd_0/SUM[23] ), .ZN(tmp_a[27]) );
  NAND3_X1 U1213 ( .A1(tmp_a_stage3[14]), .A2(tmp_a_stage3[15]), .A3(
        tmp_a_stage3[16]), .ZN(n1986) );
  INV_X1 U1214 ( .A(n1986), .ZN(n1985) );
  NAND2_X1 U1215 ( .A1(n1985), .A2(tmp_a_stage3[17]), .ZN(n1984) );
  INV_X1 U1216 ( .A(n1984), .ZN(n1983) );
  NAND2_X1 U1217 ( .A1(n1983), .A2(tmp_a_stage3[18]), .ZN(n1982) );
  INV_X1 U1218 ( .A(n1982), .ZN(n1981) );
  NAND2_X1 U1219 ( .A1(n1981), .A2(tmp_a_stage3[19]), .ZN(n1980) );
  INV_X1 U1220 ( .A(n1980), .ZN(n1979) );
  NAND2_X1 U1221 ( .A1(n1979), .A2(tmp_a_stage3[20]), .ZN(n1978) );
  INV_X1 U1222 ( .A(n1978), .ZN(n1977) );
  NAND2_X1 U1223 ( .A1(n1977), .A2(tmp_a_stage3[21]), .ZN(n1976) );
  INV_X1 U1224 ( .A(n1976), .ZN(n1975) );
  NAND2_X1 U1225 ( .A1(n1975), .A2(tmp_a_stage3[22]), .ZN(n1974) );
  INV_X1 U1226 ( .A(n1974), .ZN(n1973) );
  NAND2_X1 U1227 ( .A1(n1973), .A2(tmp_a_stage3[23]), .ZN(n1972) );
  INV_X1 U1228 ( .A(n1972), .ZN(n1971) );
  NAND2_X1 U1229 ( .A1(n1971), .A2(tmp_a_stage3[24]), .ZN(n1970) );
  INV_X1 U1230 ( .A(n1970), .ZN(n1969) );
  NAND2_X1 U1231 ( .A1(n1969), .A2(tmp_a_stage3[25]), .ZN(n1968) );
  INV_X1 U1232 ( .A(n1968), .ZN(n1967) );
  NAND2_X1 U1233 ( .A1(n1967), .A2(tmp_a_stage3[26]), .ZN(n1966) );
  INV_X1 U1234 ( .A(n1966), .ZN(n1965) );
  NAND2_X1 U1235 ( .A1(n1965), .A2(tmp_a_stage3[27]), .ZN(n1964) );
  INV_X1 U1236 ( .A(n1964), .ZN(n1963) );
  NAND2_X1 U1237 ( .A1(n1963), .A2(tmp_a_stage3[28]), .ZN(n1962) );
  INV_X1 U1238 ( .A(n1962), .ZN(n1961) );
  NAND2_X1 U1239 ( .A1(n1961), .A2(tmp_a_stage3[29]), .ZN(n1960) );
  INV_X1 U1240 ( .A(n1960), .ZN(n1959) );
  NAND2_X1 U1241 ( .A1(n1959), .A2(tmp_a_stage3[30]), .ZN(n1958) );
  XNOR2_X1 U1242 ( .A(tmp_a_stage3[31]), .B(n1958), .ZN(n1299) );
  XNOR2_X1 U1243 ( .A(tmp_i_stage3[31]), .B(\intadd_2/n1 ), .ZN(n1298) );
  XNOR2_X1 U1244 ( .A(n1299), .B(n1298), .ZN(\yi[31] ) );
  INV_X1 U1245 ( .A(\intadd_0/SUM[24] ), .ZN(tmp_a[28]) );
  INV_X1 U1246 ( .A(\intadd_0/SUM[25] ), .ZN(tmp_a[29]) );
  XNOR2_X1 U1247 ( .A(n1299), .B(tmp_r_stage3[31]), .ZN(n1300) );
  XNOR2_X1 U1248 ( .A(\intadd_1/n1 ), .B(n1300), .ZN(\yr[31] ) );
  XOR2_X1 U1249 ( .A(xbuf_real_p_imag_stage2[8]), .B(
        xbuf_real_p_imag_stage2[9]), .Z(n1461) );
  NAND2_X1 U1250 ( .A1(twiddle_real_stage2[0]), .A2(n1461), .ZN(n1658) );
  XOR2_X1 U1251 ( .A(xbuf_real_p_imag_stage2[6]), .B(n2049), .Z(n1304) );
  NOR2_X1 U1252 ( .A1(n1304), .A2(n2050), .ZN(n1716) );
  INV_X1 U1253 ( .A(n1716), .ZN(n1301) );
  NAND2_X1 U1254 ( .A1(xbuf_real_p_imag_stage2[8]), .A2(n1301), .ZN(n1664) );
  INV_X1 U1255 ( .A(n1304), .ZN(n1302) );
  XNOR2_X1 U1256 ( .A(xbuf_real_p_imag_stage2[6]), .B(
        xbuf_real_p_imag_stage2[7]), .ZN(n1305) );
  XOR2_X1 U1257 ( .A(xbuf_real_p_imag_stage2[8]), .B(
        xbuf_real_p_imag_stage2[7]), .Z(n1449) );
  NAND2_X1 U1258 ( .A1(n1302), .A2(n1449), .ZN(n1308) );
  AOI22_X1 U1259 ( .A1(twiddle_real_stage2[1]), .A2(twiddle_real_stage2[0]), 
        .B1(n2050), .B2(n2043), .ZN(n1728) );
  AOI222_X1 U1260 ( .A1(n1649), .A2(twiddle_real_stage2[0]), .B1(n1648), .B2(
        twiddle_real_stage2[1]), .C1(n1650), .C2(n1728), .ZN(n1666) );
  NAND2_X1 U1261 ( .A1(xbuf_real_p_imag_stage2[8]), .A2(n1666), .ZN(n1665) );
  NOR2_X1 U1262 ( .A1(n1664), .A2(n1665), .ZN(n1663) );
  NOR2_X1 U1263 ( .A1(twiddle_real_stage2[0]), .A2(n2043), .ZN(n1303) );
  XNOR2_X1 U1264 ( .A(twiddle_real_stage2[2]), .B(n1303), .ZN(n1731) );
  INV_X1 U1265 ( .A(n1449), .ZN(n1306) );
  NAND2_X1 U1266 ( .A1(n1305), .A2(n1304), .ZN(n1450) );
  AOI22_X1 U1267 ( .A1(twiddle_real_stage2[2]), .A2(n1648), .B1(n1651), .B2(
        twiddle_real_stage2[0]), .ZN(n1307) );
  OAI21_X1 U1268 ( .B1(n1308), .B2(n1731), .A(n1307), .ZN(n1309) );
  AOI21_X1 U1269 ( .B1(n1649), .B2(twiddle_real_stage2[1]), .A(n1309), .ZN(
        n1310) );
  XNOR2_X1 U1270 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1310), .ZN(n1662) );
  NAND2_X1 U1271 ( .A1(n1663), .A2(n1662), .ZN(n1661) );
  AOI22_X1 U1272 ( .A1(twiddle_real_stage2[2]), .A2(n1649), .B1(
        twiddle_real_stage2[3]), .B2(n1648), .ZN(n1312) );
  AOI22_X1 U1273 ( .A1(twiddle_real_stage2[1]), .A2(n1651), .B1(
        \intadd_4/SUM[0] ), .B2(n1650), .ZN(n1311) );
  NAND2_X1 U1274 ( .A1(n1312), .A2(n1311), .ZN(n1313) );
  XNOR2_X1 U1275 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1313), .ZN(n1659) );
  AOI21_X1 U1276 ( .B1(n1658), .B2(n1661), .A(n1659), .ZN(n1314) );
  INV_X1 U1277 ( .A(n1314), .ZN(\intadd_7/B[0] ) );
  NAND2_X1 U1278 ( .A1(twiddle_real_stage2[0]), .A2(n1798), .ZN(n1557) );
  XOR2_X1 U1279 ( .A(n2053), .B(xbuf_real_p_imag_stage2[12]), .Z(n1318) );
  NOR2_X1 U1280 ( .A1(n2050), .A2(n1318), .ZN(n1619) );
  XOR2_X1 U1281 ( .A(n1), .B(xbuf_real_p_imag_stage2[13]), .Z(n1319) );
  INV_X1 U1282 ( .A(n1319), .ZN(n1792) );
  INV_X1 U1283 ( .A(n1318), .ZN(n1315) );
  XNOR2_X1 U1284 ( .A(xbuf_real_p_imag_stage2[12]), .B(
        xbuf_real_p_imag_stage2[13]), .ZN(n1317) );
  AOI222_X1 U1285 ( .A1(twiddle_real_stage2[1]), .A2(n1547), .B1(
        twiddle_real_stage2[0]), .B2(n1548), .C1(n1728), .C2(n1549), .ZN(n1564) );
  NAND2_X1 U1286 ( .A1(xbuf_real_p_imag_stage2[14]), .A2(n1564), .ZN(n1316) );
  NOR2_X1 U1287 ( .A1(n1619), .A2(n1316), .ZN(n1562) );
  INV_X1 U1288 ( .A(n1548), .ZN(n1441) );
  NOR2_X1 U1289 ( .A1(n2043), .A2(n1441), .ZN(n1323) );
  NAND2_X1 U1290 ( .A1(n1318), .A2(n1317), .ZN(n1793) );
  INV_X1 U1291 ( .A(n1731), .ZN(n1320) );
  AOI22_X1 U1292 ( .A1(twiddle_real_stage2[0]), .A2(n1550), .B1(n1320), .B2(
        n1549), .ZN(n1321) );
  INV_X1 U1293 ( .A(n1321), .ZN(n1322) );
  AOI211_X1 U1294 ( .C1(n1547), .C2(twiddle_real_stage2[2]), .A(n1323), .B(
        n1322), .ZN(n1324) );
  XOR2_X1 U1295 ( .A(n1), .B(n1324), .Z(n1561) );
  NAND2_X1 U1296 ( .A1(n1562), .A2(n1561), .ZN(n1560) );
  AOI22_X1 U1297 ( .A1(twiddle_real_stage2[2]), .A2(n1548), .B1(
        twiddle_real_stage2[3]), .B2(n1547), .ZN(n1326) );
  AOI22_X1 U1298 ( .A1(twiddle_real_stage2[1]), .A2(n1550), .B1(
        \intadd_4/SUM[0] ), .B2(n1549), .ZN(n1325) );
  NAND2_X1 U1299 ( .A1(n1326), .A2(n1325), .ZN(n1327) );
  XOR2_X1 U1300 ( .A(n1), .B(n1327), .Z(n1559) );
  AOI21_X1 U1301 ( .B1(n1557), .B2(n1560), .A(n1559), .ZN(n1328) );
  INV_X1 U1302 ( .A(n1328), .ZN(\intadd_10/B[0] ) );
  AOI22_X1 U1303 ( .A1(twiddle_real_stage2[3]), .A2(n1797), .B1(n1798), .B2(
        twiddle_real_stage2[4]), .ZN(n1329) );
  XOR2_X1 U1304 ( .A(n2045), .B(n1329), .Z(\intadd_8/A[1] ) );
  INV_X1 U1305 ( .A(\intadd_8/A[1] ), .ZN(\intadd_8/B[2] ) );
  AOI22_X1 U1306 ( .A1(n1798), .A2(twiddle_real_stage2[5]), .B1(n1797), .B2(
        twiddle_real_stage2[4]), .ZN(n1330) );
  XOR2_X1 U1307 ( .A(n2045), .B(n1330), .Z(\intadd_8/A[2] ) );
  AOI22_X1 U1308 ( .A1(n1798), .A2(twiddle_real_stage2[7]), .B1(n1797), .B2(
        twiddle_real_stage2[6]), .ZN(n1331) );
  XOR2_X1 U1309 ( .A(n2045), .B(n1331), .Z(\intadd_11/A[0] ) );
  INV_X1 U1310 ( .A(\intadd_11/A[0] ), .ZN(\intadd_9/CI ) );
  AOI22_X1 U1311 ( .A1(n1798), .A2(twiddle_real_stage2[10]), .B1(n1797), .B2(
        twiddle_real_stage2[9]), .ZN(n1332) );
  XOR2_X1 U1312 ( .A(n2045), .B(n1332), .Z(\intadd_9/A[1] ) );
  INV_X1 U1313 ( .A(\intadd_9/A[1] ), .ZN(\intadd_9/B[2] ) );
  AOI22_X1 U1314 ( .A1(n1798), .A2(twiddle_real_stage2[11]), .B1(n1797), .B2(
        twiddle_real_stage2[10]), .ZN(n1333) );
  XOR2_X1 U1315 ( .A(n2045), .B(n1333), .Z(\intadd_9/A[2] ) );
  INV_X1 U1316 ( .A(\intadd_0/SUM[26] ), .ZN(tmp_a[30]) );
  INV_X1 U1317 ( .A(n1334), .ZN(\intadd_4/CI ) );
  INV_X1 U1318 ( .A(twiddle_imag[16]), .ZN(n1427) );
  INV_X1 U1319 ( .A(twiddle_real[16]), .ZN(n1430) );
  AOI22_X1 U1320 ( .A1(twiddle_real[16]), .A2(n1427), .B1(twiddle_imag[16]), 
        .B2(n1430), .ZN(n1396) );
  INV_X1 U1321 ( .A(twiddle_imag[1]), .ZN(n1359) );
  INV_X1 U1322 ( .A(twiddle_real[1]), .ZN(n1365) );
  NAND2_X1 U1323 ( .A1(twiddle_real[0]), .A2(twiddle_imag[0]), .ZN(n1360) );
  AOI222_X1 U1324 ( .A1(n1359), .A2(n1365), .B1(n1359), .B2(n1360), .C1(n1365), 
        .C2(n1360), .ZN(n1358) );
  AOI222_X1 U1325 ( .A1(twiddle_imag[2]), .A2(twiddle_real[2]), .B1(
        twiddle_imag[2]), .B2(n1358), .C1(twiddle_real[2]), .C2(n1358), .ZN(
        n1356) );
  INV_X1 U1326 ( .A(twiddle_imag[3]), .ZN(n1355) );
  INV_X1 U1327 ( .A(twiddle_real[3]), .ZN(n1370) );
  AOI222_X1 U1328 ( .A1(n1356), .A2(n1355), .B1(n1356), .B2(n1370), .C1(n1355), 
        .C2(n1370), .ZN(n1354) );
  AOI222_X1 U1329 ( .A1(twiddle_imag[4]), .A2(twiddle_real[4]), .B1(
        twiddle_imag[4]), .B2(n1354), .C1(twiddle_real[4]), .C2(n1354), .ZN(
        n1352) );
  INV_X1 U1330 ( .A(twiddle_imag[5]), .ZN(n1351) );
  INV_X1 U1331 ( .A(twiddle_real[5]), .ZN(n1374) );
  AOI222_X1 U1332 ( .A1(n1352), .A2(n1351), .B1(n1352), .B2(n1374), .C1(n1351), 
        .C2(n1374), .ZN(n1350) );
  AOI222_X1 U1333 ( .A1(twiddle_imag[6]), .A2(twiddle_real[6]), .B1(
        twiddle_imag[6]), .B2(n1350), .C1(twiddle_real[6]), .C2(n1350), .ZN(
        n1348) );
  INV_X1 U1334 ( .A(twiddle_imag[7]), .ZN(n1347) );
  INV_X1 U1335 ( .A(twiddle_real[7]), .ZN(n1378) );
  AOI222_X1 U1336 ( .A1(n1348), .A2(n1347), .B1(n1348), .B2(n1378), .C1(n1347), 
        .C2(n1378), .ZN(n1346) );
  AOI222_X1 U1337 ( .A1(twiddle_imag[8]), .A2(twiddle_real[8]), .B1(
        twiddle_imag[8]), .B2(n1346), .C1(twiddle_real[8]), .C2(n1346), .ZN(
        n1344) );
  INV_X1 U1338 ( .A(twiddle_imag[9]), .ZN(n1343) );
  INV_X1 U1339 ( .A(twiddle_real[9]), .ZN(n1382) );
  AOI222_X1 U1340 ( .A1(n1344), .A2(n1343), .B1(n1344), .B2(n1382), .C1(n1343), 
        .C2(n1382), .ZN(n1342) );
  AOI222_X1 U1341 ( .A1(twiddle_imag[10]), .A2(twiddle_real[10]), .B1(
        twiddle_imag[10]), .B2(n1342), .C1(twiddle_real[10]), .C2(n1342), .ZN(
        n1340) );
  INV_X1 U1342 ( .A(twiddle_imag[11]), .ZN(n1339) );
  INV_X1 U1343 ( .A(twiddle_real[11]), .ZN(n1386) );
  AOI222_X1 U1344 ( .A1(n1340), .A2(n1339), .B1(n1340), .B2(n1386), .C1(n1339), 
        .C2(n1386), .ZN(n1338) );
  AOI222_X1 U1345 ( .A1(twiddle_imag[12]), .A2(twiddle_real[12]), .B1(
        twiddle_imag[12]), .B2(n1338), .C1(twiddle_real[12]), .C2(n1338), .ZN(
        n1337) );
  INV_X1 U1346 ( .A(twiddle_imag[13]), .ZN(n1364) );
  INV_X1 U1347 ( .A(twiddle_real[13]), .ZN(n1390) );
  AOI222_X1 U1348 ( .A1(n1337), .A2(n1364), .B1(n1337), .B2(n1390), .C1(n1364), 
        .C2(n1390), .ZN(n1336) );
  AOI222_X1 U1349 ( .A1(twiddle_imag[14]), .A2(twiddle_real[14]), .B1(
        twiddle_imag[14]), .B2(n1336), .C1(twiddle_real[14]), .C2(n1336), .ZN(
        n1335) );
  INV_X1 U1350 ( .A(twiddle_imag[15]), .ZN(n1363) );
  INV_X1 U1351 ( .A(twiddle_real[15]), .ZN(n1394) );
  AOI222_X1 U1352 ( .A1(n1335), .A2(n1363), .B1(n1335), .B2(n1394), .C1(n1363), 
        .C2(n1394), .ZN(n1362) );
  XNOR2_X1 U1353 ( .A(n1396), .B(n1362), .ZN(twiddle_real_p_imag_stage1[16])
         );
  AOI22_X1 U1354 ( .A1(twiddle_imag[15]), .A2(twiddle_real[15]), .B1(n1394), 
        .B2(n1363), .ZN(n1397) );
  XNOR2_X1 U1355 ( .A(n1335), .B(n1397), .ZN(twiddle_real_p_imag_stage1[15])
         );
  INV_X1 U1356 ( .A(twiddle_imag[14]), .ZN(n1393) );
  XOR2_X1 U1357 ( .A(n1393), .B(twiddle_real[14]), .Z(n1400) );
  XNOR2_X1 U1358 ( .A(n1400), .B(n1336), .ZN(twiddle_real_p_imag_stage1[14])
         );
  AOI22_X1 U1359 ( .A1(twiddle_imag[13]), .A2(twiddle_real[13]), .B1(n1390), 
        .B2(n1364), .ZN(n1401) );
  XNOR2_X1 U1360 ( .A(n1337), .B(n1401), .ZN(twiddle_real_p_imag_stage1[13])
         );
  INV_X1 U1361 ( .A(twiddle_imag[12]), .ZN(n1389) );
  XOR2_X1 U1362 ( .A(n1389), .B(twiddle_real[12]), .Z(n1404) );
  XNOR2_X1 U1363 ( .A(n1404), .B(n1338), .ZN(twiddle_real_p_imag_stage1[12])
         );
  NOR2_X1 U1364 ( .A1(twiddle_real[11]), .A2(n1339), .ZN(n1387) );
  AOI21_X1 U1365 ( .B1(twiddle_real[11]), .B2(n1339), .A(n1387), .ZN(n1405) );
  XOR2_X1 U1366 ( .A(n1340), .B(n1405), .Z(twiddle_real_p_imag_stage1[11]) );
  INV_X1 U1367 ( .A(twiddle_real[10]), .ZN(n1341) );
  NAND2_X1 U1368 ( .A1(twiddle_imag[10]), .A2(n1341), .ZN(n1384) );
  OAI21_X1 U1369 ( .B1(twiddle_imag[10]), .B2(n1341), .A(n1384), .ZN(n1407) );
  XOR2_X1 U1370 ( .A(n1342), .B(n1407), .Z(twiddle_real_p_imag_stage1[10]) );
  NOR2_X1 U1371 ( .A1(twiddle_real[9]), .A2(n1343), .ZN(n1383) );
  AOI21_X1 U1372 ( .B1(twiddle_real[9]), .B2(n1343), .A(n1383), .ZN(n1409) );
  XOR2_X1 U1373 ( .A(n1344), .B(n1409), .Z(twiddle_real_p_imag_stage1[9]) );
  INV_X1 U1374 ( .A(twiddle_real[8]), .ZN(n1345) );
  NAND2_X1 U1375 ( .A1(twiddle_imag[8]), .A2(n1345), .ZN(n1380) );
  OAI21_X1 U1376 ( .B1(twiddle_imag[8]), .B2(n1345), .A(n1380), .ZN(n1411) );
  XOR2_X1 U1377 ( .A(n1346), .B(n1411), .Z(twiddle_real_p_imag_stage1[8]) );
  NOR2_X1 U1378 ( .A1(twiddle_real[7]), .A2(n1347), .ZN(n1379) );
  AOI21_X1 U1379 ( .B1(twiddle_real[7]), .B2(n1347), .A(n1379), .ZN(n1413) );
  XOR2_X1 U1380 ( .A(n1348), .B(n1413), .Z(twiddle_real_p_imag_stage1[7]) );
  INV_X1 U1381 ( .A(twiddle_real[6]), .ZN(n1349) );
  NAND2_X1 U1382 ( .A1(twiddle_imag[6]), .A2(n1349), .ZN(n1376) );
  OAI21_X1 U1383 ( .B1(twiddle_imag[6]), .B2(n1349), .A(n1376), .ZN(n1415) );
  XOR2_X1 U1384 ( .A(n1350), .B(n1415), .Z(twiddle_real_p_imag_stage1[6]) );
  NOR2_X1 U1385 ( .A1(twiddle_real[5]), .A2(n1351), .ZN(n1375) );
  AOI21_X1 U1386 ( .B1(twiddle_real[5]), .B2(n1351), .A(n1375), .ZN(n1417) );
  XOR2_X1 U1387 ( .A(n1352), .B(n1417), .Z(twiddle_real_p_imag_stage1[5]) );
  INV_X1 U1388 ( .A(twiddle_real[4]), .ZN(n1353) );
  NAND2_X1 U1389 ( .A1(twiddle_imag[4]), .A2(n1353), .ZN(n1372) );
  OAI21_X1 U1390 ( .B1(twiddle_imag[4]), .B2(n1353), .A(n1372), .ZN(n1419) );
  XOR2_X1 U1391 ( .A(n1354), .B(n1419), .Z(twiddle_real_p_imag_stage1[4]) );
  NOR2_X1 U1392 ( .A1(twiddle_real[3]), .A2(n1355), .ZN(n1371) );
  AOI21_X1 U1393 ( .B1(twiddle_real[3]), .B2(n1355), .A(n1371), .ZN(n1421) );
  XOR2_X1 U1394 ( .A(n1356), .B(n1421), .Z(twiddle_real_p_imag_stage1[3]) );
  INV_X1 U1395 ( .A(twiddle_real[2]), .ZN(n1357) );
  NAND2_X1 U1396 ( .A1(twiddle_imag[2]), .A2(n1357), .ZN(n1368) );
  OAI21_X1 U1397 ( .B1(twiddle_imag[2]), .B2(n1357), .A(n1368), .ZN(n1423) );
  XOR2_X1 U1398 ( .A(n1358), .B(n1423), .Z(twiddle_real_p_imag_stage1[2]) );
  NOR2_X1 U1399 ( .A1(twiddle_real[1]), .A2(n1359), .ZN(n1366) );
  AOI21_X1 U1400 ( .B1(twiddle_real[1]), .B2(n1359), .A(n1366), .ZN(n1425) );
  XOR2_X1 U1401 ( .A(n1425), .B(n1360), .Z(twiddle_real_p_imag_stage1[1]) );
  INV_X1 U1402 ( .A(twiddle_real[0]), .ZN(n1361) );
  NAND2_X1 U1403 ( .A1(n1361), .A2(twiddle_imag[0]), .ZN(n1426) );
  OAI21_X1 U1404 ( .B1(twiddle_imag[0]), .B2(n1361), .A(n1426), .ZN(
        twiddle_real_p_imag_stage1[0]) );
  AOI222_X1 U1405 ( .A1(n1430), .A2(n1427), .B1(n1430), .B2(n1362), .C1(n1427), 
        .C2(n1362), .ZN(twiddle_real_p_imag_stage1[17]) );
  NOR2_X1 U1406 ( .A1(twiddle_real[15]), .A2(n1363), .ZN(n1395) );
  OR2_X1 U1407 ( .A1(n1393), .A2(twiddle_real[14]), .ZN(n1392) );
  NOR2_X1 U1408 ( .A1(twiddle_real[13]), .A2(n1364), .ZN(n1391) );
  OR2_X1 U1409 ( .A1(n1389), .A2(twiddle_real[12]), .ZN(n1388) );
  INV_X1 U1410 ( .A(twiddle_imag[10]), .ZN(n1385) );
  INV_X1 U1411 ( .A(twiddle_imag[8]), .ZN(n1381) );
  INV_X1 U1412 ( .A(twiddle_imag[6]), .ZN(n1377) );
  INV_X1 U1413 ( .A(twiddle_imag[4]), .ZN(n1373) );
  INV_X1 U1414 ( .A(twiddle_imag[2]), .ZN(n1369) );
  INV_X1 U1415 ( .A(n1426), .ZN(n1367) );
  OAI22_X1 U1416 ( .A1(n1367), .A2(n1366), .B1(twiddle_imag[1]), .B2(n1365), 
        .ZN(n1424) );
  AOI22_X1 U1417 ( .A1(twiddle_real[2]), .A2(n1369), .B1(n1368), .B2(n1424), 
        .ZN(n1422) );
  OAI22_X1 U1418 ( .A1(n1371), .A2(n1422), .B1(twiddle_imag[3]), .B2(n1370), 
        .ZN(n1420) );
  AOI22_X1 U1419 ( .A1(twiddle_real[4]), .A2(n1373), .B1(n1372), .B2(n1420), 
        .ZN(n1418) );
  OAI22_X1 U1420 ( .A1(n1375), .A2(n1418), .B1(twiddle_imag[5]), .B2(n1374), 
        .ZN(n1416) );
  AOI22_X1 U1421 ( .A1(twiddle_real[6]), .A2(n1377), .B1(n1376), .B2(n1416), 
        .ZN(n1414) );
  OAI22_X1 U1422 ( .A1(n1379), .A2(n1414), .B1(twiddle_imag[7]), .B2(n1378), 
        .ZN(n1412) );
  AOI22_X1 U1423 ( .A1(twiddle_real[8]), .A2(n1381), .B1(n1380), .B2(n1412), 
        .ZN(n1410) );
  OAI22_X1 U1424 ( .A1(n1383), .A2(n1410), .B1(twiddle_imag[9]), .B2(n1382), 
        .ZN(n1408) );
  AOI22_X1 U1425 ( .A1(twiddle_real[10]), .A2(n1385), .B1(n1384), .B2(n1408), 
        .ZN(n1406) );
  OAI22_X1 U1426 ( .A1(n1387), .A2(n1406), .B1(twiddle_imag[11]), .B2(n1386), 
        .ZN(n1403) );
  AOI22_X1 U1427 ( .A1(twiddle_real[12]), .A2(n1389), .B1(n1388), .B2(n1403), 
        .ZN(n1402) );
  OAI22_X1 U1428 ( .A1(n1391), .A2(n1402), .B1(twiddle_imag[13]), .B2(n1390), 
        .ZN(n1399) );
  AOI22_X1 U1429 ( .A1(twiddle_real[14]), .A2(n1393), .B1(n1392), .B2(n1399), 
        .ZN(n1398) );
  OAI22_X1 U1430 ( .A1(n1395), .A2(n1398), .B1(twiddle_imag[15]), .B2(n1394), 
        .ZN(n1428) );
  XOR2_X1 U1431 ( .A(n1396), .B(n1428), .Z(twiddle_real_m_imag_stage1[16]) );
  XOR2_X1 U1432 ( .A(n1398), .B(n1397), .Z(twiddle_real_m_imag_stage1[15]) );
  XOR2_X1 U1433 ( .A(n1400), .B(n1399), .Z(twiddle_real_m_imag_stage1[14]) );
  XOR2_X1 U1434 ( .A(n1402), .B(n1401), .Z(twiddle_real_m_imag_stage1[13]) );
  XOR2_X1 U1435 ( .A(n1404), .B(n1403), .Z(twiddle_real_m_imag_stage1[12]) );
  XNOR2_X1 U1436 ( .A(n1406), .B(n1405), .ZN(twiddle_real_m_imag_stage1[11])
         );
  XNOR2_X1 U1437 ( .A(n1408), .B(n1407), .ZN(twiddle_real_m_imag_stage1[10])
         );
  XNOR2_X1 U1438 ( .A(n1410), .B(n1409), .ZN(twiddle_real_m_imag_stage1[9]) );
  XNOR2_X1 U1439 ( .A(n1412), .B(n1411), .ZN(twiddle_real_m_imag_stage1[8]) );
  XNOR2_X1 U1440 ( .A(n1414), .B(n1413), .ZN(twiddle_real_m_imag_stage1[7]) );
  XNOR2_X1 U1441 ( .A(n1416), .B(n1415), .ZN(twiddle_real_m_imag_stage1[6]) );
  XNOR2_X1 U1442 ( .A(n1418), .B(n1417), .ZN(twiddle_real_m_imag_stage1[5]) );
  XNOR2_X1 U1443 ( .A(n1420), .B(n1419), .ZN(twiddle_real_m_imag_stage1[4]) );
  XNOR2_X1 U1444 ( .A(n1422), .B(n1421), .ZN(twiddle_real_m_imag_stage1[3]) );
  XNOR2_X1 U1445 ( .A(n1424), .B(n1423), .ZN(twiddle_real_m_imag_stage1[2]) );
  XOR2_X1 U1446 ( .A(n1426), .B(n1425), .Z(twiddle_real_m_imag_stage1[1]) );
  NOR2_X1 U1447 ( .A1(twiddle_real[16]), .A2(n1427), .ZN(n1429) );
  OAI22_X1 U1448 ( .A1(twiddle_imag[16]), .A2(n1430), .B1(n1429), .B2(n1428), 
        .ZN(twiddle_real_m_imag_stage1[17]) );
  AOI22_X1 U1449 ( .A1(n1798), .A2(twiddle_real_stage2[13]), .B1(n1797), .B2(
        twiddle_real_stage2[12]), .ZN(n1431) );
  XOR2_X1 U1450 ( .A(n2045), .B(n1431), .Z(n1791) );
  AOI22_X1 U1451 ( .A1(n1550), .A2(twiddle_real_stage2[15]), .B1(n1549), .B2(
        \intadd_4/n1 ), .ZN(n1432) );
  NAND2_X1 U1452 ( .A1(n1547), .A2(twiddle_real_stage2[16]), .ZN(n1439) );
  OAI211_X1 U1453 ( .C1(n2062), .C2(n1441), .A(n1432), .B(n1439), .ZN(n1433)
         );
  XOR2_X1 U1454 ( .A(n1), .B(n1433), .Z(n1436) );
  AOI22_X1 U1455 ( .A1(n1798), .A2(twiddle_real_stage2[14]), .B1(n1797), .B2(
        twiddle_real_stage2[13]), .ZN(n1434) );
  XOR2_X1 U1456 ( .A(n1434), .B(xbuf_real_p_imag_stage2[16]), .Z(n1435) );
  FA_X1 U1457 ( .A(n1791), .B(n1436), .CI(n1435), .CO(\intadd_0/A[26] ), .S(
        \intadd_0/A[25] ) );
  AOI22_X1 U1458 ( .A1(n1798), .A2(twiddle_real_stage2[12]), .B1(n1797), .B2(
        twiddle_real_stage2[11]), .ZN(n1437) );
  XOR2_X1 U1459 ( .A(n1437), .B(xbuf_real_p_imag_stage2[16]), .Z(n1447) );
  XOR2_X1 U1460 ( .A(xbuf_real_p_imag_stage2[9]), .B(
        xbuf_real_p_imag_stage2[10]), .Z(n1462) );
  NOR2_X1 U1461 ( .A1(n1461), .A2(n1462), .ZN(n1463) );
  XOR2_X1 U1462 ( .A(n2053), .B(xbuf_real_p_imag_stage2[10]), .Z(n1464) );
  AOI21_X1 U1463 ( .B1(n1463), .B2(n1464), .A(n2062), .ZN(n1438) );
  XOR2_X1 U1464 ( .A(n1438), .B(xbuf_real_p_imag_stage2[11]), .Z(n1446) );
  INV_X1 U1465 ( .A(\intadd_9/A[2] ), .ZN(n1445) );
  AOI22_X1 U1466 ( .A1(n1550), .A2(twiddle_real_stage2[14]), .B1(n1549), .B2(
        \intadd_4/SUM[13] ), .ZN(n1440) );
  OAI211_X1 U1467 ( .C1(n2066), .C2(n1441), .A(n1440), .B(n1439), .ZN(n1442)
         );
  XOR2_X1 U1468 ( .A(n1), .B(n1442), .Z(n1443) );
  FA_X1 U1469 ( .A(n1791), .B(n1444), .CI(n1443), .CO(\intadd_0/B[25] ), .S(
        \intadd_0/A[24] ) );
  FA_X1 U1470 ( .A(n1447), .B(n1446), .CI(n1445), .CO(n1444), .S(
        \intadd_9/A[3] ) );
  AOI22_X1 U1471 ( .A1(n1798), .A2(twiddle_real_stage2[9]), .B1(n1797), .B2(
        twiddle_real_stage2[8]), .ZN(n1448) );
  XOR2_X1 U1472 ( .A(n1448), .B(xbuf_real_p_imag_stage2[16]), .Z(
        \intadd_9/A[0] ) );
  OAI21_X1 U1473 ( .B1(n1450), .B2(n1449), .A(twiddle_real_stage2[16]), .ZN(
        n1451) );
  XNOR2_X1 U1474 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1451), .ZN(
        \intadd_9/B[0] ) );
  AOI22_X1 U1475 ( .A1(n1548), .A2(twiddle_real_stage2[12]), .B1(n1547), .B2(
        twiddle_real_stage2[13]), .ZN(n1453) );
  AOI22_X1 U1476 ( .A1(n1550), .A2(twiddle_real_stage2[11]), .B1(n1549), .B2(
        \intadd_4/SUM[10] ), .ZN(n1452) );
  NAND2_X1 U1477 ( .A1(n1453), .A2(n1452), .ZN(n1454) );
  XOR2_X1 U1478 ( .A(n1), .B(n1454), .Z(\intadd_9/B[1] ) );
  AOI22_X1 U1479 ( .A1(n1548), .A2(twiddle_real_stage2[14]), .B1(n1547), .B2(
        twiddle_real_stage2[15]), .ZN(n1456) );
  AOI22_X1 U1480 ( .A1(n1550), .A2(twiddle_real_stage2[13]), .B1(n1549), .B2(
        \intadd_4/SUM[12] ), .ZN(n1455) );
  NAND2_X1 U1481 ( .A1(n1456), .A2(n1455), .ZN(n1457) );
  XOR2_X1 U1482 ( .A(n1), .B(n1457), .Z(\intadd_9/B[3] ) );
  AOI22_X1 U1483 ( .A1(n1548), .A2(twiddle_real_stage2[13]), .B1(n1547), .B2(
        twiddle_real_stage2[14]), .ZN(n1459) );
  AOI22_X1 U1484 ( .A1(n1550), .A2(twiddle_real_stage2[12]), .B1(n1549), .B2(
        \intadd_4/SUM[11] ), .ZN(n1458) );
  NAND2_X1 U1485 ( .A1(n1459), .A2(n1458), .ZN(n1460) );
  XOR2_X1 U1486 ( .A(n1), .B(n1460), .Z(n1470) );
  INV_X1 U1487 ( .A(n1461), .ZN(n1465) );
  NAND2_X1 U1488 ( .A1(n1465), .A2(n1462), .ZN(n1502) );
  INV_X1 U1489 ( .A(n1464), .ZN(n1466) );
  NAND2_X1 U1490 ( .A1(n1466), .A2(n1463), .ZN(n1571) );
  INV_X1 U1491 ( .A(n1571), .ZN(n1589) );
  AOI22_X1 U1492 ( .A1(n1589), .A2(twiddle_real_stage2[15]), .B1(n1588), .B2(
        \intadd_4/n1 ), .ZN(n1467) );
  NAND2_X1 U1493 ( .A1(n1586), .A2(twiddle_real_stage2[16]), .ZN(n1478) );
  OAI211_X1 U1494 ( .C1(n2062), .C2(n1502), .A(n1467), .B(n1478), .ZN(n1468)
         );
  XOR2_X1 U1495 ( .A(n2053), .B(n1468), .Z(n1469) );
  FA_X1 U1496 ( .A(\intadd_9/SUM[2] ), .B(n1470), .CI(n1469), .CO(
        \intadd_0/B[23] ), .S(\intadd_0/A[22] ) );
  AOI22_X1 U1497 ( .A1(n1548), .A2(twiddle_real_stage2[10]), .B1(n1547), .B2(
        twiddle_real_stage2[11]), .ZN(n1472) );
  AOI22_X1 U1498 ( .A1(n1550), .A2(twiddle_real_stage2[9]), .B1(n1549), .B2(
        \intadd_4/SUM[8] ), .ZN(n1471) );
  NAND2_X1 U1499 ( .A1(n1472), .A2(n1471), .ZN(n1473) );
  XOR2_X1 U1500 ( .A(n1), .B(n1473), .Z(\intadd_11/B[0] ) );
  AOI22_X1 U1501 ( .A1(n1798), .A2(twiddle_real_stage2[8]), .B1(n1797), .B2(
        twiddle_real_stage2[7]), .ZN(n1474) );
  XOR2_X1 U1502 ( .A(n1474), .B(xbuf_real_p_imag_stage2[16]), .Z(
        \intadd_11/CI ) );
  AOI22_X1 U1503 ( .A1(n1548), .A2(twiddle_real_stage2[11]), .B1(n1547), .B2(
        twiddle_real_stage2[12]), .ZN(n1476) );
  AOI22_X1 U1504 ( .A1(n1550), .A2(twiddle_real_stage2[10]), .B1(n1549), .B2(
        \intadd_4/SUM[9] ), .ZN(n1475) );
  NAND2_X1 U1505 ( .A1(n1476), .A2(n1475), .ZN(n1477) );
  XOR2_X1 U1506 ( .A(n1), .B(n1477), .Z(\intadd_11/B[1] ) );
  AOI22_X1 U1507 ( .A1(n1589), .A2(twiddle_real_stage2[14]), .B1(n1588), .B2(
        \intadd_4/SUM[13] ), .ZN(n1479) );
  OAI211_X1 U1508 ( .C1(n2066), .C2(n1502), .A(n1479), .B(n1478), .ZN(n1480)
         );
  XOR2_X1 U1509 ( .A(n2053), .B(n1480), .Z(\intadd_11/B[2] ) );
  INV_X1 U1510 ( .A(\intadd_8/A[2] ), .ZN(n1490) );
  XOR2_X1 U1511 ( .A(n1784), .B(xbuf_real_p_imag_stage2[3]), .Z(n1605) );
  XNOR2_X1 U1512 ( .A(xbuf_real_p_imag_stage2[3]), .B(
        xbuf_real_p_imag_stage2[4]), .ZN(n1604) );
  AND2_X1 U1513 ( .A1(n1605), .A2(n1604), .ZN(n1603) );
  XOR2_X1 U1514 ( .A(n2049), .B(xbuf_real_p_imag_stage2[4]), .Z(n1602) );
  AOI21_X1 U1515 ( .B1(n1603), .B2(n1602), .A(n2062), .ZN(n1481) );
  XOR2_X1 U1516 ( .A(n1481), .B(xbuf_real_p_imag_stage2[5]), .Z(n1489) );
  AOI22_X1 U1517 ( .A1(n1798), .A2(twiddle_real_stage2[6]), .B1(n1797), .B2(
        twiddle_real_stage2[5]), .ZN(n1482) );
  XOR2_X1 U1518 ( .A(n1482), .B(xbuf_real_p_imag_stage2[16]), .Z(n1488) );
  AOI22_X1 U1519 ( .A1(n1548), .A2(twiddle_real_stage2[9]), .B1(n1547), .B2(
        twiddle_real_stage2[10]), .ZN(n1484) );
  AOI22_X1 U1520 ( .A1(n1550), .A2(twiddle_real_stage2[8]), .B1(n1549), .B2(
        \intadd_4/SUM[7] ), .ZN(n1483) );
  NAND2_X1 U1521 ( .A1(n1484), .A2(n1483), .ZN(n1485) );
  XOR2_X1 U1522 ( .A(n1), .B(n1485), .Z(n1486) );
  FA_X1 U1523 ( .A(\intadd_11/A[0] ), .B(n1487), .CI(n1486), .CO(
        \intadd_8/B[5] ), .S(\intadd_8/A[4] ) );
  FA_X1 U1524 ( .A(n1490), .B(n1489), .CI(n1488), .CO(n1487), .S(
        \intadd_8/A[3] ) );
  AOI21_X1 U1525 ( .B1(n2042), .B2(n2044), .A(n2062), .ZN(n1491) );
  OAI22_X1 U1526 ( .A1(n2062), .A2(n1784), .B1(xbuf_real_p_imag_stage2[2]), 
        .B2(n1491), .ZN(n1494) );
  AOI22_X1 U1527 ( .A1(twiddle_real_stage2[2]), .A2(n1797), .B1(
        twiddle_real_stage2[3]), .B2(n1798), .ZN(n1492) );
  XOR2_X1 U1528 ( .A(n2045), .B(n1492), .Z(n1493) );
  NOR2_X1 U1529 ( .A1(n1494), .A2(n1493), .ZN(\intadd_8/B[1] ) );
  AOI21_X1 U1530 ( .B1(n1494), .B2(n1493), .A(\intadd_8/B[1] ), .ZN(
        \intadd_8/A[0] ) );
  AOI22_X1 U1531 ( .A1(n1548), .A2(twiddle_real_stage2[5]), .B1(n1547), .B2(
        twiddle_real_stage2[6]), .ZN(n1496) );
  AOI22_X1 U1532 ( .A1(n1550), .A2(twiddle_real_stage2[4]), .B1(n1549), .B2(
        \intadd_4/SUM[3] ), .ZN(n1495) );
  NAND2_X1 U1533 ( .A1(n1496), .A2(n1495), .ZN(n1497) );
  XOR2_X1 U1534 ( .A(n1), .B(n1497), .Z(\intadd_8/B[0] ) );
  NAND2_X1 U1535 ( .A1(xbuf_real_p_imag_stage2[16]), .A2(n1557), .ZN(n1539) );
  AOI22_X1 U1536 ( .A1(twiddle_real_stage2[1]), .A2(n1798), .B1(
        twiddle_real_stage2[0]), .B2(n1797), .ZN(n1541) );
  NAND2_X1 U1537 ( .A1(xbuf_real_p_imag_stage2[16]), .A2(n1541), .ZN(n1540) );
  NOR2_X1 U1538 ( .A1(n1539), .A2(n1540), .ZN(n1538) );
  AOI22_X1 U1539 ( .A1(twiddle_real_stage2[2]), .A2(n1798), .B1(
        twiddle_real_stage2[1]), .B2(n1797), .ZN(n1498) );
  XOR2_X1 U1540 ( .A(n2045), .B(n1498), .Z(n1537) );
  NAND2_X1 U1541 ( .A1(n1538), .A2(n1537), .ZN(\intadd_8/CI ) );
  AOI22_X1 U1542 ( .A1(n1548), .A2(twiddle_real_stage2[8]), .B1(n1547), .B2(
        twiddle_real_stage2[9]), .ZN(n1500) );
  AOI22_X1 U1543 ( .A1(n1550), .A2(twiddle_real_stage2[7]), .B1(n1549), .B2(
        \intadd_4/SUM[6] ), .ZN(n1499) );
  NAND2_X1 U1544 ( .A1(n1500), .A2(n1499), .ZN(n1501) );
  XOR2_X1 U1545 ( .A(n1), .B(n1501), .Z(\intadd_8/B[3] ) );
  INV_X1 U1546 ( .A(n1502), .ZN(n1587) );
  AOI22_X1 U1547 ( .A1(n1587), .A2(twiddle_real_stage2[12]), .B1(n1586), .B2(
        twiddle_real_stage2[13]), .ZN(n1504) );
  AOI22_X1 U1548 ( .A1(n1589), .A2(twiddle_real_stage2[11]), .B1(n1588), .B2(
        \intadd_4/SUM[10] ), .ZN(n1503) );
  NAND2_X1 U1549 ( .A1(n1504), .A2(n1503), .ZN(n1505) );
  XOR2_X1 U1550 ( .A(n2053), .B(n1505), .Z(\intadd_8/B[4] ) );
  AOI22_X1 U1551 ( .A1(n1587), .A2(twiddle_real_stage2[14]), .B1(n1586), .B2(
        twiddle_real_stage2[15]), .ZN(n1507) );
  AOI22_X1 U1552 ( .A1(n1589), .A2(twiddle_real_stage2[13]), .B1(n1588), .B2(
        \intadd_4/SUM[12] ), .ZN(n1506) );
  NAND2_X1 U1553 ( .A1(n1507), .A2(n1506), .ZN(n1508) );
  XOR2_X1 U1554 ( .A(n2053), .B(n1508), .Z(\intadd_8/B[6] ) );
  AOI22_X1 U1555 ( .A1(n1587), .A2(twiddle_real_stage2[13]), .B1(n1586), .B2(
        twiddle_real_stage2[14]), .ZN(n1510) );
  AOI22_X1 U1556 ( .A1(n1589), .A2(twiddle_real_stage2[12]), .B1(n1588), .B2(
        \intadd_4/SUM[11] ), .ZN(n1509) );
  NAND2_X1 U1557 ( .A1(n1510), .A2(n1509), .ZN(n1511) );
  XOR2_X1 U1558 ( .A(n2053), .B(n1511), .Z(n1515) );
  INV_X1 U1559 ( .A(n1649), .ZN(n1527) );
  AOI22_X1 U1560 ( .A1(n1651), .A2(twiddle_real_stage2[15]), .B1(n1650), .B2(
        \intadd_4/n1 ), .ZN(n1512) );
  NAND2_X1 U1561 ( .A1(n1648), .A2(twiddle_real_stage2[16]), .ZN(n1525) );
  OAI211_X1 U1562 ( .C1(n2062), .C2(n1527), .A(n1512), .B(n1525), .ZN(n1513)
         );
  XNOR2_X1 U1563 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1513), .ZN(n1514) );
  FA_X1 U1564 ( .A(\intadd_8/SUM[5] ), .B(n1515), .CI(n1514), .CO(
        \intadd_0/B[20] ), .S(\intadd_0/A[19] ) );
  AOI22_X1 U1565 ( .A1(n1587), .A2(twiddle_real_stage2[10]), .B1(n1586), .B2(
        twiddle_real_stage2[11]), .ZN(n1517) );
  AOI22_X1 U1566 ( .A1(n1589), .A2(twiddle_real_stage2[9]), .B1(n1588), .B2(
        \intadd_4/SUM[8] ), .ZN(n1516) );
  NAND2_X1 U1567 ( .A1(n1517), .A2(n1516), .ZN(n1518) );
  XOR2_X1 U1568 ( .A(n2053), .B(n1518), .Z(\intadd_12/B[0] ) );
  AOI22_X1 U1569 ( .A1(n1548), .A2(twiddle_real_stage2[7]), .B1(n1547), .B2(
        twiddle_real_stage2[8]), .ZN(n1520) );
  AOI22_X1 U1570 ( .A1(n1550), .A2(twiddle_real_stage2[6]), .B1(n1549), .B2(
        \intadd_4/SUM[5] ), .ZN(n1519) );
  NAND2_X1 U1571 ( .A1(n1520), .A2(n1519), .ZN(n1521) );
  XOR2_X1 U1572 ( .A(n1), .B(n1521), .Z(\intadd_12/CI ) );
  AOI22_X1 U1573 ( .A1(n1587), .A2(twiddle_real_stage2[11]), .B1(n1586), .B2(
        twiddle_real_stage2[12]), .ZN(n1523) );
  AOI22_X1 U1574 ( .A1(n1589), .A2(twiddle_real_stage2[10]), .B1(n1588), .B2(
        \intadd_4/SUM[9] ), .ZN(n1522) );
  NAND2_X1 U1575 ( .A1(n1523), .A2(n1522), .ZN(n1524) );
  XOR2_X1 U1576 ( .A(n2053), .B(n1524), .Z(\intadd_12/B[1] ) );
  AOI22_X1 U1577 ( .A1(n1651), .A2(twiddle_real_stage2[14]), .B1(n1650), .B2(
        \intadd_4/SUM[13] ), .ZN(n1526) );
  OAI211_X1 U1578 ( .C1(n2066), .C2(n1527), .A(n1526), .B(n1525), .ZN(n1528)
         );
  XNOR2_X1 U1579 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1528), .ZN(
        \intadd_12/B[2] ) );
  AOI22_X1 U1580 ( .A1(n1587), .A2(twiddle_real_stage2[9]), .B1(n1586), .B2(
        twiddle_real_stage2[10]), .ZN(n1530) );
  AOI22_X1 U1581 ( .A1(n1589), .A2(twiddle_real_stage2[8]), .B1(n1588), .B2(
        \intadd_4/SUM[7] ), .ZN(n1529) );
  NAND2_X1 U1582 ( .A1(n1530), .A2(n1529), .ZN(n1531) );
  XOR2_X1 U1583 ( .A(n2053), .B(n1531), .Z(n1536) );
  AOI22_X1 U1584 ( .A1(n1548), .A2(twiddle_real_stage2[6]), .B1(n1547), .B2(
        twiddle_real_stage2[7]), .ZN(n1533) );
  AOI22_X1 U1585 ( .A1(n1550), .A2(twiddle_real_stage2[5]), .B1(n1549), .B2(
        \intadd_4/SUM[4] ), .ZN(n1532) );
  NAND2_X1 U1586 ( .A1(n1533), .A2(n1532), .ZN(n1534) );
  XOR2_X1 U1587 ( .A(n1), .B(n1534), .Z(n1535) );
  FA_X1 U1588 ( .A(\intadd_8/SUM[1] ), .B(n1536), .CI(n1535), .CO(
        \intadd_6/B[7] ), .S(\intadd_6/A[6] ) );
  OAI21_X1 U1589 ( .B1(n1538), .B2(n1537), .A(\intadd_8/CI ), .ZN(
        \intadd_10/A[1] ) );
  INV_X1 U1590 ( .A(n1538), .ZN(n1543) );
  OAI211_X1 U1591 ( .C1(xbuf_real_p_imag_stage2[16]), .C2(n1541), .A(n1540), 
        .B(n1539), .ZN(n1542) );
  NAND2_X1 U1592 ( .A1(n1543), .A2(n1542), .ZN(\intadd_10/A[0] ) );
  AOI22_X1 U1593 ( .A1(twiddle_real_stage2[3]), .A2(n1548), .B1(n1547), .B2(
        twiddle_real_stage2[4]), .ZN(n1545) );
  AOI22_X1 U1594 ( .A1(twiddle_real_stage2[2]), .A2(n1550), .B1(n1549), .B2(
        \intadd_4/SUM[1] ), .ZN(n1544) );
  NAND2_X1 U1595 ( .A1(n1545), .A2(n1544), .ZN(n1546) );
  XOR2_X1 U1596 ( .A(n1), .B(n1546), .Z(\intadd_10/CI ) );
  AOI22_X1 U1597 ( .A1(n1548), .A2(twiddle_real_stage2[4]), .B1(n1547), .B2(
        twiddle_real_stage2[5]), .ZN(n1552) );
  AOI22_X1 U1598 ( .A1(twiddle_real_stage2[3]), .A2(n1550), .B1(n1549), .B2(
        \intadd_4/SUM[2] ), .ZN(n1551) );
  NAND2_X1 U1599 ( .A1(n1552), .A2(n1551), .ZN(n1553) );
  XOR2_X1 U1600 ( .A(n1), .B(n1553), .Z(\intadd_10/B[1] ) );
  AOI22_X1 U1601 ( .A1(n1587), .A2(twiddle_real_stage2[8]), .B1(n1586), .B2(
        twiddle_real_stage2[9]), .ZN(n1555) );
  AOI22_X1 U1602 ( .A1(n1589), .A2(twiddle_real_stage2[7]), .B1(n1588), .B2(
        \intadd_4/SUM[6] ), .ZN(n1554) );
  NAND2_X1 U1603 ( .A1(n1555), .A2(n1554), .ZN(n1556) );
  XOR2_X1 U1604 ( .A(n2053), .B(n1556), .Z(\intadd_10/B[2] ) );
  XNOR2_X1 U1605 ( .A(n1557), .B(n1560), .ZN(n1558) );
  XNOR2_X1 U1606 ( .A(n1559), .B(n1558), .ZN(\intadd_6/A[2] ) );
  OAI21_X1 U1607 ( .B1(n1562), .B2(n1561), .A(n1560), .ZN(\intadd_6/A[1] ) );
  NAND2_X1 U1608 ( .A1(n1619), .A2(xbuf_real_p_imag_stage2[14]), .ZN(n1563) );
  XNOR2_X1 U1609 ( .A(n1564), .B(n1563), .ZN(\intadd_6/A[0] ) );
  INV_X1 U1610 ( .A(n1588), .ZN(n1566) );
  AOI22_X1 U1611 ( .A1(twiddle_real_stage2[1]), .A2(n1587), .B1(
        twiddle_real_stage2[0]), .B2(n1589), .ZN(n1565) );
  OAI21_X1 U1612 ( .B1(n1731), .B2(n1566), .A(n1565), .ZN(n1567) );
  AOI21_X1 U1613 ( .B1(twiddle_real_stage2[2]), .B2(n1586), .A(n1567), .ZN(
        n1568) );
  XNOR2_X1 U1614 ( .A(n2053), .B(n1568), .ZN(n1624) );
  AOI222_X1 U1615 ( .A1(n1588), .A2(n1728), .B1(n1586), .B2(
        twiddle_real_stage2[1]), .C1(twiddle_real_stage2[0]), .C2(n1587), .ZN(
        n1625) );
  AND2_X1 U1616 ( .A1(xbuf_real_p_imag_stage2[11]), .A2(n1625), .ZN(n1569) );
  NAND2_X1 U1617 ( .A1(n1658), .A2(n1569), .ZN(n1623) );
  NOR2_X1 U1618 ( .A1(n1624), .A2(n1623), .ZN(n1622) );
  AOI22_X1 U1619 ( .A1(twiddle_real_stage2[3]), .A2(n1586), .B1(
        \intadd_4/SUM[0] ), .B2(n1588), .ZN(n1570) );
  OAI21_X1 U1620 ( .B1(n2043), .B2(n1571), .A(n1570), .ZN(n1572) );
  AOI21_X1 U1621 ( .B1(twiddle_real_stage2[2]), .B2(n1587), .A(n1572), .ZN(
        n1573) );
  XOR2_X1 U1622 ( .A(n2053), .B(n1573), .Z(n1620) );
  OAI21_X1 U1623 ( .B1(n1619), .B2(n1622), .A(n1620), .ZN(\intadd_6/B[0] ) );
  AOI22_X1 U1624 ( .A1(twiddle_real_stage2[3]), .A2(n1587), .B1(n1586), .B2(
        twiddle_real_stage2[4]), .ZN(n1575) );
  AOI22_X1 U1625 ( .A1(twiddle_real_stage2[2]), .A2(n1589), .B1(n1588), .B2(
        \intadd_4/SUM[1] ), .ZN(n1574) );
  NAND2_X1 U1626 ( .A1(n1575), .A2(n1574), .ZN(n1576) );
  XOR2_X1 U1627 ( .A(n2053), .B(n1576), .Z(\intadd_6/CI ) );
  AOI22_X1 U1628 ( .A1(n1587), .A2(twiddle_real_stage2[4]), .B1(n1586), .B2(
        twiddle_real_stage2[5]), .ZN(n1578) );
  AOI22_X1 U1629 ( .A1(twiddle_real_stage2[3]), .A2(n1589), .B1(n1588), .B2(
        \intadd_4/SUM[2] ), .ZN(n1577) );
  NAND2_X1 U1630 ( .A1(n1578), .A2(n1577), .ZN(n1579) );
  XOR2_X1 U1631 ( .A(n2053), .B(n1579), .Z(\intadd_6/B[1] ) );
  AOI22_X1 U1632 ( .A1(n1587), .A2(twiddle_real_stage2[5]), .B1(n1586), .B2(
        twiddle_real_stage2[6]), .ZN(n1581) );
  AOI22_X1 U1633 ( .A1(n1589), .A2(twiddle_real_stage2[4]), .B1(n1588), .B2(
        \intadd_4/SUM[3] ), .ZN(n1580) );
  NAND2_X1 U1634 ( .A1(n1581), .A2(n1580), .ZN(n1582) );
  XOR2_X1 U1635 ( .A(n2053), .B(n1582), .Z(\intadd_6/B[2] ) );
  AOI22_X1 U1636 ( .A1(n1587), .A2(twiddle_real_stage2[6]), .B1(n1586), .B2(
        twiddle_real_stage2[7]), .ZN(n1584) );
  AOI22_X1 U1637 ( .A1(n1589), .A2(twiddle_real_stage2[5]), .B1(n1588), .B2(
        \intadd_4/SUM[4] ), .ZN(n1583) );
  NAND2_X1 U1638 ( .A1(n1584), .A2(n1583), .ZN(n1585) );
  XOR2_X1 U1639 ( .A(n2053), .B(n1585), .Z(\intadd_6/B[3] ) );
  AOI22_X1 U1640 ( .A1(n1587), .A2(twiddle_real_stage2[7]), .B1(n1586), .B2(
        twiddle_real_stage2[8]), .ZN(n1591) );
  AOI22_X1 U1641 ( .A1(n1589), .A2(twiddle_real_stage2[6]), .B1(n1588), .B2(
        \intadd_4/SUM[5] ), .ZN(n1590) );
  NAND2_X1 U1642 ( .A1(n1591), .A2(n1590), .ZN(n1592) );
  XOR2_X1 U1643 ( .A(n2053), .B(n1592), .Z(\intadd_6/B[4] ) );
  AOI22_X1 U1644 ( .A1(n1649), .A2(twiddle_real_stage2[11]), .B1(n1648), .B2(
        twiddle_real_stage2[12]), .ZN(n1594) );
  AOI22_X1 U1645 ( .A1(n1651), .A2(twiddle_real_stage2[10]), .B1(n1650), .B2(
        \intadd_4/SUM[9] ), .ZN(n1593) );
  NAND2_X1 U1646 ( .A1(n1594), .A2(n1593), .ZN(n1595) );
  XNOR2_X1 U1647 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1595), .ZN(
        \intadd_6/B[5] ) );
  AOI22_X1 U1648 ( .A1(n1649), .A2(twiddle_real_stage2[14]), .B1(n1648), .B2(
        twiddle_real_stage2[15]), .ZN(n1597) );
  AOI22_X1 U1649 ( .A1(n1651), .A2(twiddle_real_stage2[13]), .B1(n1650), .B2(
        \intadd_4/SUM[12] ), .ZN(n1596) );
  NAND2_X1 U1650 ( .A1(n1597), .A2(n1596), .ZN(n1598) );
  XNOR2_X1 U1651 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1598), .ZN(
        \intadd_6/B[8] ) );
  AOI22_X1 U1652 ( .A1(n1649), .A2(twiddle_real_stage2[13]), .B1(n1648), .B2(
        twiddle_real_stage2[14]), .ZN(n1600) );
  AOI22_X1 U1653 ( .A1(n1651), .A2(twiddle_real_stage2[12]), .B1(n1650), .B2(
        \intadd_4/SUM[11] ), .ZN(n1599) );
  NAND2_X1 U1654 ( .A1(n1600), .A2(n1599), .ZN(n1601) );
  XNOR2_X1 U1655 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1601), .ZN(n1610) );
  INV_X1 U1656 ( .A(n1602), .ZN(n1606) );
  NAND2_X1 U1657 ( .A1(n1606), .A2(n1603), .ZN(n1674) );
  INV_X1 U1658 ( .A(n1605), .ZN(n1669) );
  NAND2_X1 U1659 ( .A1(n1669), .A2(n1606), .ZN(n1671) );
  AOI22_X1 U1660 ( .A1(n1708), .A2(twiddle_real_stage2[16]), .B1(n1709), .B2(
        \intadd_4/n1 ), .ZN(n1607) );
  NAND2_X1 U1661 ( .A1(n1707), .A2(twiddle_real_stage2[16]), .ZN(n1611) );
  OAI211_X1 U1662 ( .C1(n2066), .C2(n1674), .A(n1607), .B(n1611), .ZN(n1608)
         );
  XOR2_X1 U1663 ( .A(n2049), .B(n1608), .Z(n1609) );
  FA_X1 U1664 ( .A(\intadd_6/SUM[7] ), .B(n1610), .CI(n1609), .CO(
        \intadd_0/B[17] ), .S(\intadd_0/A[16] ) );
  AOI22_X1 U1665 ( .A1(n1708), .A2(twiddle_real_stage2[15]), .B1(n1709), .B2(
        \intadd_4/SUM[13] ), .ZN(n1612) );
  OAI211_X1 U1666 ( .C1(n2069), .C2(n1674), .A(n1612), .B(n1611), .ZN(n1613)
         );
  XOR2_X1 U1667 ( .A(n2049), .B(n1613), .Z(n1618) );
  AOI22_X1 U1668 ( .A1(n1649), .A2(twiddle_real_stage2[12]), .B1(n1648), .B2(
        twiddle_real_stage2[13]), .ZN(n1615) );
  AOI22_X1 U1669 ( .A1(n1651), .A2(twiddle_real_stage2[11]), .B1(n1650), .B2(
        \intadd_4/SUM[10] ), .ZN(n1614) );
  NAND2_X1 U1670 ( .A1(n1615), .A2(n1614), .ZN(n1616) );
  XNOR2_X1 U1671 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1616), .ZN(n1617) );
  FA_X1 U1672 ( .A(\intadd_6/SUM[6] ), .B(n1618), .CI(n1617), .CO(
        \intadd_0/B[16] ), .S(\intadd_0/A[15] ) );
  XOR2_X1 U1673 ( .A(n1620), .B(n1619), .Z(n1621) );
  XNOR2_X1 U1674 ( .A(n1622), .B(n1621), .ZN(\intadd_7/A[2] ) );
  XNOR2_X1 U1675 ( .A(n1624), .B(n1623), .ZN(\intadd_7/A[1] ) );
  NOR2_X1 U1676 ( .A1(n1658), .A2(n2053), .ZN(n1626) );
  XOR2_X1 U1677 ( .A(n1626), .B(n1625), .Z(\intadd_7/A[0] ) );
  AOI22_X1 U1678 ( .A1(n1649), .A2(twiddle_real_stage2[3]), .B1(n1648), .B2(
        twiddle_real_stage2[4]), .ZN(n1628) );
  AOI22_X1 U1679 ( .A1(twiddle_real_stage2[2]), .A2(n1651), .B1(n1650), .B2(
        \intadd_4/SUM[1] ), .ZN(n1627) );
  NAND2_X1 U1680 ( .A1(n1628), .A2(n1627), .ZN(n1629) );
  XNOR2_X1 U1681 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1629), .ZN(
        \intadd_7/CI ) );
  AOI22_X1 U1682 ( .A1(n1649), .A2(twiddle_real_stage2[4]), .B1(n1648), .B2(
        twiddle_real_stage2[5]), .ZN(n1631) );
  AOI22_X1 U1683 ( .A1(twiddle_real_stage2[3]), .A2(n1651), .B1(n1650), .B2(
        \intadd_4/SUM[2] ), .ZN(n1630) );
  NAND2_X1 U1684 ( .A1(n1631), .A2(n1630), .ZN(n1632) );
  XNOR2_X1 U1685 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1632), .ZN(
        \intadd_7/B[1] ) );
  AOI22_X1 U1686 ( .A1(n1649), .A2(twiddle_real_stage2[5]), .B1(n1648), .B2(
        twiddle_real_stage2[6]), .ZN(n1634) );
  AOI22_X1 U1687 ( .A1(n1651), .A2(twiddle_real_stage2[4]), .B1(n1650), .B2(
        \intadd_4/SUM[3] ), .ZN(n1633) );
  NAND2_X1 U1688 ( .A1(n1634), .A2(n1633), .ZN(n1635) );
  XNOR2_X1 U1689 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1635), .ZN(
        \intadd_7/B[2] ) );
  AOI22_X1 U1690 ( .A1(n1649), .A2(twiddle_real_stage2[6]), .B1(n1648), .B2(
        twiddle_real_stage2[7]), .ZN(n1637) );
  AOI22_X1 U1691 ( .A1(n1651), .A2(twiddle_real_stage2[5]), .B1(n1650), .B2(
        \intadd_4/SUM[4] ), .ZN(n1636) );
  NAND2_X1 U1692 ( .A1(n1637), .A2(n1636), .ZN(n1638) );
  XNOR2_X1 U1693 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1638), .ZN(
        \intadd_7/B[3] ) );
  AOI22_X1 U1694 ( .A1(n1649), .A2(twiddle_real_stage2[7]), .B1(n1648), .B2(
        twiddle_real_stage2[8]), .ZN(n1640) );
  AOI22_X1 U1695 ( .A1(n1651), .A2(twiddle_real_stage2[6]), .B1(n1650), .B2(
        \intadd_4/SUM[5] ), .ZN(n1639) );
  NAND2_X1 U1696 ( .A1(n1640), .A2(n1639), .ZN(n1641) );
  XNOR2_X1 U1697 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1641), .ZN(
        \intadd_7/B[4] ) );
  AOI22_X1 U1698 ( .A1(n1649), .A2(twiddle_real_stage2[8]), .B1(n1648), .B2(
        twiddle_real_stage2[9]), .ZN(n1643) );
  AOI22_X1 U1699 ( .A1(n1651), .A2(twiddle_real_stage2[7]), .B1(n1650), .B2(
        \intadd_4/SUM[6] ), .ZN(n1642) );
  NAND2_X1 U1700 ( .A1(n1643), .A2(n1642), .ZN(n1644) );
  XNOR2_X1 U1701 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1644), .ZN(
        \intadd_7/B[5] ) );
  AOI22_X1 U1702 ( .A1(n1649), .A2(twiddle_real_stage2[9]), .B1(n1648), .B2(
        twiddle_real_stage2[10]), .ZN(n1646) );
  AOI22_X1 U1703 ( .A1(n1651), .A2(twiddle_real_stage2[8]), .B1(n1650), .B2(
        \intadd_4/SUM[7] ), .ZN(n1645) );
  NAND2_X1 U1704 ( .A1(n1646), .A2(n1645), .ZN(n1647) );
  XNOR2_X1 U1705 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1647), .ZN(
        \intadd_7/B[6] ) );
  AOI22_X1 U1706 ( .A1(n1649), .A2(twiddle_real_stage2[10]), .B1(n1648), .B2(
        twiddle_real_stage2[11]), .ZN(n1653) );
  AOI22_X1 U1707 ( .A1(n1651), .A2(twiddle_real_stage2[9]), .B1(n1650), .B2(
        \intadd_4/SUM[8] ), .ZN(n1652) );
  NAND2_X1 U1708 ( .A1(n1653), .A2(n1652), .ZN(n1654) );
  XNOR2_X1 U1709 ( .A(xbuf_real_p_imag_stage2[8]), .B(n1654), .ZN(
        \intadd_7/B[7] ) );
  AOI22_X1 U1710 ( .A1(n1708), .A2(twiddle_real_stage2[14]), .B1(n1707), .B2(
        twiddle_real_stage2[15]), .ZN(n1656) );
  AOI22_X1 U1711 ( .A1(n1710), .A2(twiddle_real_stage2[13]), .B1(n1709), .B2(
        \intadd_4/SUM[12] ), .ZN(n1655) );
  NAND2_X1 U1712 ( .A1(n1656), .A2(n1655), .ZN(n1657) );
  XOR2_X1 U1713 ( .A(n2049), .B(n1657), .Z(\intadd_7/B[8] ) );
  XNOR2_X1 U1714 ( .A(n1659), .B(n1658), .ZN(n1660) );
  XNOR2_X1 U1715 ( .A(n1661), .B(n1660), .ZN(\intadd_5/A[2] ) );
  OAI21_X1 U1716 ( .B1(n1663), .B2(n1662), .A(n1661), .ZN(\intadd_5/A[1] ) );
  INV_X1 U1717 ( .A(n1663), .ZN(n1668) );
  OAI211_X1 U1718 ( .C1(xbuf_real_p_imag_stage2[8]), .C2(n1666), .A(n1665), 
        .B(n1664), .ZN(n1667) );
  NAND2_X1 U1719 ( .A1(n1668), .A2(n1667), .ZN(\intadd_5/A[0] ) );
  AOI222_X1 U1720 ( .A1(twiddle_real_stage2[1]), .A2(n1707), .B1(
        twiddle_real_stage2[0]), .B2(n1708), .C1(n1728), .C2(n1709), .ZN(n1724) );
  NAND2_X1 U1721 ( .A1(twiddle_real_stage2[0]), .A2(n1669), .ZN(n1735) );
  NAND3_X1 U1722 ( .A1(xbuf_real_p_imag_stage2[5]), .A2(n1724), .A3(n1735), 
        .ZN(n1718) );
  AOI22_X1 U1723 ( .A1(twiddle_real_stage2[2]), .A2(n1707), .B1(
        twiddle_real_stage2[0]), .B2(n1710), .ZN(n1670) );
  OAI21_X1 U1724 ( .B1(n1731), .B2(n1671), .A(n1670), .ZN(n1672) );
  AOI21_X1 U1725 ( .B1(twiddle_real_stage2[1]), .B2(n1708), .A(n1672), .ZN(
        n1720) );
  NAND2_X1 U1726 ( .A1(xbuf_real_p_imag_stage2[5]), .A2(n1720), .ZN(n1719) );
  AOI22_X1 U1727 ( .A1(twiddle_real_stage2[3]), .A2(n1707), .B1(
        \intadd_4/SUM[0] ), .B2(n1709), .ZN(n1673) );
  OAI21_X1 U1728 ( .B1(n2043), .B2(n1674), .A(n1673), .ZN(n1675) );
  AOI21_X1 U1729 ( .B1(twiddle_real_stage2[2]), .B2(n1708), .A(n1675), .ZN(
        n1676) );
  XOR2_X1 U1730 ( .A(n2049), .B(n1676), .Z(n1714) );
  OAI21_X1 U1731 ( .B1(n1716), .B2(n1717), .A(n1714), .ZN(\intadd_5/B[0] ) );
  AOI22_X1 U1732 ( .A1(twiddle_real_stage2[3]), .A2(n1708), .B1(n1707), .B2(
        twiddle_real_stage2[4]), .ZN(n1678) );
  AOI22_X1 U1733 ( .A1(twiddle_real_stage2[2]), .A2(n1710), .B1(n1709), .B2(
        \intadd_4/SUM[1] ), .ZN(n1677) );
  NAND2_X1 U1734 ( .A1(n1678), .A2(n1677), .ZN(n1679) );
  XOR2_X1 U1735 ( .A(n2049), .B(n1679), .Z(\intadd_5/CI ) );
  AOI22_X1 U1736 ( .A1(n1708), .A2(twiddle_real_stage2[4]), .B1(n1707), .B2(
        twiddle_real_stage2[5]), .ZN(n1681) );
  AOI22_X1 U1737 ( .A1(twiddle_real_stage2[3]), .A2(n1710), .B1(n1709), .B2(
        \intadd_4/SUM[2] ), .ZN(n1680) );
  NAND2_X1 U1738 ( .A1(n1681), .A2(n1680), .ZN(n1682) );
  XOR2_X1 U1739 ( .A(n2049), .B(n1682), .Z(\intadd_5/B[1] ) );
  AOI22_X1 U1740 ( .A1(n1708), .A2(twiddle_real_stage2[5]), .B1(n1707), .B2(
        twiddle_real_stage2[6]), .ZN(n1684) );
  AOI22_X1 U1741 ( .A1(n1710), .A2(twiddle_real_stage2[4]), .B1(n1709), .B2(
        \intadd_4/SUM[3] ), .ZN(n1683) );
  NAND2_X1 U1742 ( .A1(n1684), .A2(n1683), .ZN(n1685) );
  XOR2_X1 U1743 ( .A(n2049), .B(n1685), .Z(\intadd_5/B[2] ) );
  AOI22_X1 U1744 ( .A1(n1708), .A2(twiddle_real_stage2[6]), .B1(n1707), .B2(
        twiddle_real_stage2[7]), .ZN(n1687) );
  AOI22_X1 U1745 ( .A1(n1710), .A2(twiddle_real_stage2[5]), .B1(n1709), .B2(
        \intadd_4/SUM[4] ), .ZN(n1686) );
  NAND2_X1 U1746 ( .A1(n1687), .A2(n1686), .ZN(n1688) );
  XOR2_X1 U1747 ( .A(n2049), .B(n1688), .Z(\intadd_5/B[3] ) );
  AOI22_X1 U1748 ( .A1(n1708), .A2(twiddle_real_stage2[7]), .B1(n1707), .B2(
        twiddle_real_stage2[8]), .ZN(n1690) );
  AOI22_X1 U1749 ( .A1(n1710), .A2(twiddle_real_stage2[6]), .B1(n1709), .B2(
        \intadd_4/SUM[5] ), .ZN(n1689) );
  NAND2_X1 U1750 ( .A1(n1690), .A2(n1689), .ZN(n1691) );
  XOR2_X1 U1751 ( .A(n2049), .B(n1691), .Z(\intadd_5/B[4] ) );
  AOI22_X1 U1752 ( .A1(n1708), .A2(twiddle_real_stage2[8]), .B1(n1707), .B2(
        twiddle_real_stage2[9]), .ZN(n1693) );
  AOI22_X1 U1753 ( .A1(n1710), .A2(twiddle_real_stage2[7]), .B1(n1709), .B2(
        \intadd_4/SUM[6] ), .ZN(n1692) );
  NAND2_X1 U1754 ( .A1(n1693), .A2(n1692), .ZN(n1694) );
  XOR2_X1 U1755 ( .A(n2049), .B(n1694), .Z(\intadd_5/B[5] ) );
  AOI22_X1 U1756 ( .A1(n1708), .A2(twiddle_real_stage2[9]), .B1(n1707), .B2(
        twiddle_real_stage2[10]), .ZN(n1696) );
  AOI22_X1 U1757 ( .A1(n1710), .A2(twiddle_real_stage2[8]), .B1(n1709), .B2(
        \intadd_4/SUM[7] ), .ZN(n1695) );
  NAND2_X1 U1758 ( .A1(n1696), .A2(n1695), .ZN(n1697) );
  XOR2_X1 U1759 ( .A(n2049), .B(n1697), .Z(\intadd_5/B[6] ) );
  AOI22_X1 U1760 ( .A1(n1708), .A2(twiddle_real_stage2[10]), .B1(n1707), .B2(
        twiddle_real_stage2[11]), .ZN(n1699) );
  AOI22_X1 U1761 ( .A1(n1710), .A2(twiddle_real_stage2[9]), .B1(n1709), .B2(
        \intadd_4/SUM[8] ), .ZN(n1698) );
  NAND2_X1 U1762 ( .A1(n1699), .A2(n1698), .ZN(n1700) );
  XOR2_X1 U1763 ( .A(n2049), .B(n1700), .Z(\intadd_5/B[7] ) );
  AOI22_X1 U1764 ( .A1(n1708), .A2(twiddle_real_stage2[11]), .B1(n1707), .B2(
        twiddle_real_stage2[12]), .ZN(n1702) );
  AOI22_X1 U1765 ( .A1(n1710), .A2(twiddle_real_stage2[10]), .B1(n1709), .B2(
        \intadd_4/SUM[9] ), .ZN(n1701) );
  NAND2_X1 U1766 ( .A1(n1702), .A2(n1701), .ZN(n1703) );
  XOR2_X1 U1767 ( .A(n2049), .B(n1703), .Z(\intadd_5/B[8] ) );
  AOI22_X1 U1768 ( .A1(n1708), .A2(twiddle_real_stage2[12]), .B1(n1707), .B2(
        twiddle_real_stage2[13]), .ZN(n1705) );
  AOI22_X1 U1769 ( .A1(n1710), .A2(twiddle_real_stage2[11]), .B1(n1709), .B2(
        \intadd_4/SUM[10] ), .ZN(n1704) );
  NAND2_X1 U1770 ( .A1(n1705), .A2(n1704), .ZN(n1706) );
  XOR2_X1 U1771 ( .A(n2049), .B(n1706), .Z(\intadd_5/B[9] ) );
  AOI22_X1 U1772 ( .A1(n1708), .A2(twiddle_real_stage2[13]), .B1(n1707), .B2(
        twiddle_real_stage2[14]), .ZN(n1712) );
  AOI22_X1 U1773 ( .A1(n1710), .A2(twiddle_real_stage2[12]), .B1(n1709), .B2(
        \intadd_4/SUM[11] ), .ZN(n1711) );
  NAND2_X1 U1774 ( .A1(n1712), .A2(n1711), .ZN(n1713) );
  XOR2_X1 U1775 ( .A(n2049), .B(n1713), .Z(\intadd_5/B[10] ) );
  XNOR2_X1 U1776 ( .A(n1714), .B(n1717), .ZN(n1715) );
  XOR2_X1 U1777 ( .A(n1716), .B(n1715), .Z(\intadd_0/A[2] ) );
  INV_X1 U1778 ( .A(n1717), .ZN(n1722) );
  OAI211_X1 U1779 ( .C1(xbuf_real_p_imag_stage2[5]), .C2(n1720), .A(n1719), 
        .B(n1718), .ZN(n1721) );
  NAND2_X1 U1780 ( .A1(n1722), .A2(n1721), .ZN(\intadd_0/A[1] ) );
  NOR2_X1 U1781 ( .A1(n2049), .A2(n1735), .ZN(n1723) );
  XOR2_X1 U1782 ( .A(n1724), .B(n1723), .Z(\intadd_0/A[0] ) );
  NOR2_X1 U1783 ( .A1(n2050), .A2(n2044), .ZN(tmp_a[0]) );
  NAND2_X1 U1784 ( .A1(n2044), .A2(xbuf_real_p_imag_stage2[1]), .ZN(n1781) );
  INV_X1 U1785 ( .A(n1781), .ZN(n1770) );
  OAI221_X1 U1786 ( .B1(n1784), .B2(xbuf_real_p_imag_stage2[1]), .C1(
        xbuf_real_p_imag_stage2[2]), .C2(n2042), .A(xbuf_real_p_imag_stage2[0]), .ZN(n1782) );
  AOI22_X1 U1787 ( .A1(twiddle_real_stage2[2]), .A2(n1770), .B1(
        twiddle_real_stage2[3]), .B2(n1774), .ZN(n1726) );
  OAI221_X1 U1788 ( .B1(n1784), .B2(n2042), .C1(xbuf_real_p_imag_stage2[2]), 
        .C2(xbuf_real_p_imag_stage2[1]), .A(xbuf_real_p_imag_stage2[0]), .ZN(
        n1732) );
  AOI22_X1 U1789 ( .A1(twiddle_real_stage2[1]), .A2(n1779), .B1(
        \intadd_4/SUM[0] ), .B2(n1778), .ZN(n1725) );
  NAND2_X1 U1790 ( .A1(n1726), .A2(n1725), .ZN(n1727) );
  XNOR2_X1 U1791 ( .A(n1784), .B(n1727), .ZN(n1786) );
  AOI222_X1 U1792 ( .A1(n1774), .A2(twiddle_real_stage2[1]), .B1(
        twiddle_real_stage2[0]), .B2(n1770), .C1(n1778), .C2(n1728), .ZN(n1790) );
  NAND2_X1 U1793 ( .A1(xbuf_real_p_imag_stage2[2]), .A2(n1790), .ZN(n1729) );
  NOR2_X1 U1794 ( .A1(tmp_a[0]), .A2(n1729), .ZN(n1788) );
  AOI22_X1 U1795 ( .A1(twiddle_real_stage2[2]), .A2(n1774), .B1(
        twiddle_real_stage2[0]), .B2(n1779), .ZN(n1730) );
  OAI21_X1 U1796 ( .B1(n1732), .B2(n1731), .A(n1730), .ZN(n1733) );
  AOI21_X1 U1797 ( .B1(twiddle_real_stage2[1]), .B2(n1770), .A(n1733), .ZN(
        n1734) );
  XOR2_X1 U1798 ( .A(n1784), .B(n1734), .Z(n1787) );
  NAND2_X1 U1799 ( .A1(n1788), .A2(n1787), .ZN(n1736) );
  NAND2_X1 U1800 ( .A1(n1736), .A2(n1735), .ZN(n1785) );
  NAND2_X1 U1801 ( .A1(n1786), .A2(n1785), .ZN(\intadd_0/B[0] ) );
  AOI22_X1 U1802 ( .A1(twiddle_real_stage2[3]), .A2(n1770), .B1(
        twiddle_real_stage2[4]), .B2(n1774), .ZN(n1738) );
  AOI22_X1 U1803 ( .A1(twiddle_real_stage2[2]), .A2(n1779), .B1(
        \intadd_4/SUM[1] ), .B2(n1778), .ZN(n1737) );
  NAND2_X1 U1804 ( .A1(n1738), .A2(n1737), .ZN(n1739) );
  XOR2_X1 U1805 ( .A(n1784), .B(n1739), .Z(\intadd_0/CI ) );
  AOI22_X1 U1806 ( .A1(n1770), .A2(twiddle_real_stage2[4]), .B1(
        twiddle_real_stage2[5]), .B2(n1774), .ZN(n1741) );
  AOI22_X1 U1807 ( .A1(twiddle_real_stage2[3]), .A2(n1779), .B1(
        \intadd_4/SUM[2] ), .B2(n1778), .ZN(n1740) );
  NAND2_X1 U1808 ( .A1(n1741), .A2(n1740), .ZN(n1742) );
  XOR2_X1 U1809 ( .A(n1784), .B(n1742), .Z(\intadd_0/B[1] ) );
  AOI22_X1 U1810 ( .A1(n1770), .A2(twiddle_real_stage2[5]), .B1(
        twiddle_real_stage2[6]), .B2(n1774), .ZN(n1744) );
  AOI22_X1 U1811 ( .A1(n1779), .A2(twiddle_real_stage2[4]), .B1(
        \intadd_4/SUM[3] ), .B2(n1778), .ZN(n1743) );
  NAND2_X1 U1812 ( .A1(n1744), .A2(n1743), .ZN(n1745) );
  XOR2_X1 U1813 ( .A(n1784), .B(n1745), .Z(\intadd_0/B[2] ) );
  AOI22_X1 U1814 ( .A1(n1770), .A2(twiddle_real_stage2[6]), .B1(
        twiddle_real_stage2[7]), .B2(n1774), .ZN(n1747) );
  AOI22_X1 U1815 ( .A1(n1779), .A2(twiddle_real_stage2[5]), .B1(
        \intadd_4/SUM[4] ), .B2(n1778), .ZN(n1746) );
  NAND2_X1 U1816 ( .A1(n1747), .A2(n1746), .ZN(n1748) );
  XOR2_X1 U1817 ( .A(n1784), .B(n1748), .Z(\intadd_0/B[3] ) );
  AOI22_X1 U1818 ( .A1(n1770), .A2(twiddle_real_stage2[7]), .B1(
        twiddle_real_stage2[8]), .B2(n1774), .ZN(n1750) );
  AOI22_X1 U1819 ( .A1(n1779), .A2(twiddle_real_stage2[6]), .B1(
        \intadd_4/SUM[5] ), .B2(n1778), .ZN(n1749) );
  NAND2_X1 U1820 ( .A1(n1750), .A2(n1749), .ZN(n1751) );
  XOR2_X1 U1821 ( .A(n1784), .B(n1751), .Z(\intadd_0/B[4] ) );
  AOI22_X1 U1822 ( .A1(n1770), .A2(twiddle_real_stage2[8]), .B1(
        twiddle_real_stage2[9]), .B2(n1774), .ZN(n1753) );
  AOI22_X1 U1823 ( .A1(n1779), .A2(twiddle_real_stage2[7]), .B1(
        \intadd_4/SUM[6] ), .B2(n1778), .ZN(n1752) );
  NAND2_X1 U1824 ( .A1(n1753), .A2(n1752), .ZN(n1754) );
  XOR2_X1 U1825 ( .A(n1784), .B(n1754), .Z(\intadd_0/B[5] ) );
  AOI22_X1 U1826 ( .A1(n1770), .A2(twiddle_real_stage2[9]), .B1(
        twiddle_real_stage2[10]), .B2(n1774), .ZN(n1756) );
  AOI22_X1 U1827 ( .A1(n1779), .A2(twiddle_real_stage2[8]), .B1(
        \intadd_4/SUM[7] ), .B2(n1778), .ZN(n1755) );
  NAND2_X1 U1828 ( .A1(n1756), .A2(n1755), .ZN(n1757) );
  XOR2_X1 U1829 ( .A(n1784), .B(n1757), .Z(\intadd_0/B[6] ) );
  AOI22_X1 U1830 ( .A1(n1770), .A2(twiddle_real_stage2[10]), .B1(
        twiddle_real_stage2[11]), .B2(n1774), .ZN(n1759) );
  AOI22_X1 U1831 ( .A1(n1779), .A2(twiddle_real_stage2[9]), .B1(
        \intadd_4/SUM[8] ), .B2(n1778), .ZN(n1758) );
  NAND2_X1 U1832 ( .A1(n1759), .A2(n1758), .ZN(n1760) );
  XOR2_X1 U1833 ( .A(n1784), .B(n1760), .Z(\intadd_0/B[7] ) );
  AOI22_X1 U1834 ( .A1(n1770), .A2(twiddle_real_stage2[11]), .B1(
        twiddle_real_stage2[12]), .B2(n1774), .ZN(n1762) );
  AOI22_X1 U1835 ( .A1(n1779), .A2(twiddle_real_stage2[10]), .B1(
        \intadd_4/SUM[9] ), .B2(n1778), .ZN(n1761) );
  NAND2_X1 U1836 ( .A1(n1762), .A2(n1761), .ZN(n1763) );
  XOR2_X1 U1837 ( .A(n1784), .B(n1763), .Z(\intadd_0/B[8] ) );
  AOI22_X1 U1838 ( .A1(n1770), .A2(twiddle_real_stage2[12]), .B1(
        twiddle_real_stage2[13]), .B2(n1774), .ZN(n1765) );
  AOI22_X1 U1839 ( .A1(n1779), .A2(twiddle_real_stage2[11]), .B1(
        \intadd_4/SUM[10] ), .B2(n1778), .ZN(n1764) );
  NAND2_X1 U1840 ( .A1(n1765), .A2(n1764), .ZN(n1766) );
  XOR2_X1 U1841 ( .A(n1784), .B(n1766), .Z(\intadd_0/B[9] ) );
  AOI22_X1 U1842 ( .A1(n1770), .A2(twiddle_real_stage2[13]), .B1(
        twiddle_real_stage2[14]), .B2(n1774), .ZN(n1768) );
  AOI22_X1 U1843 ( .A1(n1779), .A2(twiddle_real_stage2[12]), .B1(
        \intadd_4/SUM[11] ), .B2(n1778), .ZN(n1767) );
  NAND2_X1 U1844 ( .A1(n1768), .A2(n1767), .ZN(n1769) );
  XOR2_X1 U1845 ( .A(n1784), .B(n1769), .Z(\intadd_0/B[10] ) );
  AOI22_X1 U1846 ( .A1(n1770), .A2(twiddle_real_stage2[14]), .B1(
        twiddle_real_stage2[15]), .B2(n1774), .ZN(n1772) );
  AOI22_X1 U1847 ( .A1(n1779), .A2(twiddle_real_stage2[13]), .B1(
        \intadd_4/SUM[12] ), .B2(n1778), .ZN(n1771) );
  NAND2_X1 U1848 ( .A1(n1772), .A2(n1771), .ZN(n1773) );
  XOR2_X1 U1849 ( .A(n1784), .B(n1773), .Z(\intadd_0/B[11] ) );
  AOI22_X1 U1850 ( .A1(n1779), .A2(twiddle_real_stage2[14]), .B1(
        \intadd_4/SUM[13] ), .B2(n1778), .ZN(n1776) );
  NAND2_X1 U1851 ( .A1(twiddle_real_stage2[16]), .A2(n1774), .ZN(n1775) );
  OAI211_X1 U1852 ( .C1(n2066), .C2(n1781), .A(n1776), .B(n1775), .ZN(n1777)
         );
  XOR2_X1 U1853 ( .A(n1784), .B(n1777), .Z(\intadd_0/B[12] ) );
  AOI22_X1 U1854 ( .A1(n1779), .A2(twiddle_real_stage2[15]), .B1(\intadd_4/n1 ), .B2(n1778), .ZN(n1780) );
  OAI221_X1 U1855 ( .B1(n2062), .B2(n1782), .C1(n2062), .C2(n1781), .A(n1780), 
        .ZN(n1783) );
  XOR2_X1 U1856 ( .A(n1784), .B(n1783), .Z(\intadd_0/B[13] ) );
  XOR2_X1 U1857 ( .A(n1786), .B(n1785), .Z(tmp_a[3]) );
  XOR2_X1 U1858 ( .A(n1788), .B(n1787), .Z(tmp_a[2]) );
  NAND2_X1 U1859 ( .A1(tmp_a[0]), .A2(xbuf_real_p_imag_stage2[2]), .ZN(n1789)
         );
  XOR2_X1 U1860 ( .A(n1790), .B(n1789), .Z(tmp_a[1]) );
  INV_X1 U1861 ( .A(n1791), .ZN(n1802) );
  OAI21_X1 U1862 ( .B1(n1793), .B2(n1792), .A(twiddle_real_stage2[16]), .ZN(
        n1794) );
  XOR2_X1 U1863 ( .A(n1), .B(n1794), .Z(n1801) );
  AOI22_X1 U1864 ( .A1(n1798), .A2(twiddle_real_stage2[15]), .B1(n1797), .B2(
        twiddle_real_stage2[14]), .ZN(n1796) );
  XOR2_X1 U1865 ( .A(n1796), .B(xbuf_real_p_imag_stage2[16]), .Z(n1800) );
  AOI22_X1 U1866 ( .A1(n1798), .A2(twiddle_real_stage2[16]), .B1(n1797), .B2(
        twiddle_real_stage2[15]), .ZN(n1799) );
  XOR2_X1 U1867 ( .A(n1799), .B(\intadd_0/n1 ), .Z(n1805) );
  FA_X1 U1868 ( .A(n1802), .B(n1801), .CI(n1800), .CO(n1803), .S(
        \intadd_0/B[26] ) );
  XOR2_X1 U1869 ( .A(n2045), .B(n1803), .Z(n1804) );
  XNOR2_X1 U1870 ( .A(n1805), .B(n1804), .ZN(tmp_a[31]) );
  AOI22_X1 U1871 ( .A1(opa_real[15]), .A2(n1851), .B1(opb_real[15]), .B2(n1852), .ZN(n1955) );
  AND2_X1 U1872 ( .A1(opa_real[0]), .A2(opb_real[0]), .ZN(n1849) );
  AOI222_X1 U1873 ( .A1(opb_real[1]), .A2(opa_real[1]), .B1(opb_real[1]), .B2(
        n1849), .C1(opa_real[1]), .C2(n1849), .ZN(n1847) );
  AOI222_X1 U1874 ( .A1(n1847), .A2(n1806), .B1(n1847), .B2(n1846), .C1(n1806), 
        .C2(n1846), .ZN(n1844) );
  AOI222_X1 U1875 ( .A1(opb_real[3]), .A2(opa_real[3]), .B1(opb_real[3]), .B2(
        n1844), .C1(opa_real[3]), .C2(n1844), .ZN(n1841) );
  AOI222_X1 U1876 ( .A1(n1841), .A2(n1807), .B1(n1841), .B2(n1840), .C1(n1807), 
        .C2(n1840), .ZN(n1838) );
  AOI222_X1 U1877 ( .A1(opb_real[5]), .A2(opa_real[5]), .B1(opb_real[5]), .B2(
        n1838), .C1(opa_real[5]), .C2(n1838), .ZN(n1835) );
  AOI222_X1 U1878 ( .A1(n1835), .A2(n1808), .B1(n1835), .B2(n1834), .C1(n1808), 
        .C2(n1834), .ZN(n1832) );
  AOI222_X1 U1879 ( .A1(opb_real[7]), .A2(opa_real[7]), .B1(opb_real[7]), .B2(
        n1832), .C1(opa_real[7]), .C2(n1832), .ZN(n1829) );
  AOI222_X1 U1880 ( .A1(n1829), .A2(n1809), .B1(n1829), .B2(n1828), .C1(n1809), 
        .C2(n1828), .ZN(n1826) );
  AOI222_X1 U1881 ( .A1(opb_real[9]), .A2(opa_real[9]), .B1(opb_real[9]), .B2(
        n1826), .C1(opa_real[9]), .C2(n1826), .ZN(n1823) );
  AOI222_X1 U1882 ( .A1(n1823), .A2(n1810), .B1(n1823), .B2(n1822), .C1(n1810), 
        .C2(n1822), .ZN(n1820) );
  AOI222_X1 U1883 ( .A1(opb_real[11]), .A2(opa_real[11]), .B1(opb_real[11]), 
        .B2(n1820), .C1(opa_real[11]), .C2(n1820), .ZN(n1818) );
  AOI222_X1 U1884 ( .A1(n1818), .A2(n1816), .B1(n1818), .B2(n1817), .C1(n1816), 
        .C2(n1817), .ZN(n1815) );
  AOI222_X1 U1885 ( .A1(opb_real[13]), .A2(opa_real[13]), .B1(opb_real[13]), 
        .B2(n1815), .C1(opa_real[13]), .C2(n1815), .ZN(n1813) );
  AOI222_X1 U1886 ( .A1(n1813), .A2(n1811), .B1(n1813), .B2(n1812), .C1(n1811), 
        .C2(n1812), .ZN(n1850) );
  XNOR2_X1 U1887 ( .A(n1955), .B(n1850), .ZN(N92) );
  AOI22_X1 U1888 ( .A1(opb_real[14]), .A2(opa_real[14]), .B1(n1812), .B2(n1811), .ZN(n1953) );
  XNOR2_X1 U1889 ( .A(n1813), .B(n1953), .ZN(N91) );
  XOR2_X1 U1890 ( .A(opb_real[13]), .B(n1814), .Z(n1951) );
  XNOR2_X1 U1891 ( .A(n1951), .B(n1815), .ZN(N90) );
  AOI22_X1 U1892 ( .A1(opb_real[12]), .A2(opa_real[12]), .B1(n1817), .B2(n1816), .ZN(n1949) );
  XNOR2_X1 U1893 ( .A(n1818), .B(n1949), .ZN(N89) );
  XOR2_X1 U1894 ( .A(opb_real[11]), .B(n1819), .Z(n1947) );
  XNOR2_X1 U1895 ( .A(n1947), .B(n1820), .ZN(N88) );
  OAI21_X1 U1896 ( .B1(opb_real[10]), .B2(n1822), .A(n1821), .ZN(n1944) );
  XNOR2_X1 U1897 ( .A(n1823), .B(n1944), .ZN(N87) );
  AOI21_X1 U1898 ( .B1(opa_real[9]), .B2(n1825), .A(n1824), .ZN(n1942) );
  XNOR2_X1 U1899 ( .A(n1942), .B(n1826), .ZN(N86) );
  OAI21_X1 U1900 ( .B1(opb_real[8]), .B2(n1828), .A(n1827), .ZN(n1940) );
  XNOR2_X1 U1901 ( .A(n1829), .B(n1940), .ZN(N85) );
  AOI21_X1 U1902 ( .B1(opa_real[7]), .B2(n1831), .A(n1830), .ZN(n1938) );
  XNOR2_X1 U1903 ( .A(n1938), .B(n1832), .ZN(N84) );
  OAI21_X1 U1904 ( .B1(opb_real[6]), .B2(n1834), .A(n1833), .ZN(n1936) );
  XNOR2_X1 U1905 ( .A(n1835), .B(n1936), .ZN(N83) );
  AOI21_X1 U1906 ( .B1(opa_real[5]), .B2(n1837), .A(n1836), .ZN(n1934) );
  XNOR2_X1 U1907 ( .A(n1934), .B(n1838), .ZN(N82) );
  OAI21_X1 U1908 ( .B1(opb_real[4]), .B2(n1840), .A(n1839), .ZN(n1932) );
  XNOR2_X1 U1909 ( .A(n1841), .B(n1932), .ZN(N81) );
  AOI21_X1 U1910 ( .B1(opa_real[3]), .B2(n1843), .A(n1842), .ZN(n1930) );
  XNOR2_X1 U1911 ( .A(n1930), .B(n1844), .ZN(N80) );
  OAI21_X1 U1912 ( .B1(opb_real[2]), .B2(n1846), .A(n1845), .ZN(n1926) );
  XNOR2_X1 U1913 ( .A(n1847), .B(n1926), .ZN(N79) );
  XNOR2_X1 U1914 ( .A(n1849), .B(n1848), .ZN(N78) );
  AOI222_X1 U1915 ( .A1(n1852), .A2(n1851), .B1(n1852), .B2(n1850), .C1(n1851), 
        .C2(n1850), .ZN(N93) );
  AOI22_X1 U1916 ( .A1(opa_imag[15]), .A2(n1898), .B1(opb_imag[15]), .B2(n1899), .ZN(n1901) );
  AND2_X1 U1917 ( .A1(opa_imag[0]), .A2(opb_imag[0]), .ZN(n1896) );
  AOI222_X1 U1918 ( .A1(opb_imag[1]), .A2(opa_imag[1]), .B1(opb_imag[1]), .B2(
        n1896), .C1(opa_imag[1]), .C2(n1896), .ZN(n1894) );
  AOI222_X1 U1919 ( .A1(n1894), .A2(n1853), .B1(n1894), .B2(n1893), .C1(n1853), 
        .C2(n1893), .ZN(n1891) );
  AOI222_X1 U1920 ( .A1(opb_imag[3]), .A2(opa_imag[3]), .B1(opb_imag[3]), .B2(
        n1891), .C1(opa_imag[3]), .C2(n1891), .ZN(n1888) );
  AOI222_X1 U1921 ( .A1(n1888), .A2(n1854), .B1(n1888), .B2(n1887), .C1(n1854), 
        .C2(n1887), .ZN(n1885) );
  AOI222_X1 U1922 ( .A1(opb_imag[5]), .A2(opa_imag[5]), .B1(opb_imag[5]), .B2(
        n1885), .C1(opa_imag[5]), .C2(n1885), .ZN(n1882) );
  AOI222_X1 U1923 ( .A1(n1882), .A2(n1855), .B1(n1882), .B2(n1881), .C1(n1855), 
        .C2(n1881), .ZN(n1879) );
  AOI222_X1 U1924 ( .A1(opb_imag[7]), .A2(opa_imag[7]), .B1(opb_imag[7]), .B2(
        n1879), .C1(opa_imag[7]), .C2(n1879), .ZN(n1876) );
  AOI222_X1 U1925 ( .A1(n1876), .A2(n1856), .B1(n1876), .B2(n1875), .C1(n1856), 
        .C2(n1875), .ZN(n1873) );
  AOI222_X1 U1926 ( .A1(opb_imag[9]), .A2(opa_imag[9]), .B1(opb_imag[9]), .B2(
        n1873), .C1(opa_imag[9]), .C2(n1873), .ZN(n1870) );
  AOI222_X1 U1927 ( .A1(n1870), .A2(n1857), .B1(n1870), .B2(n1869), .C1(n1857), 
        .C2(n1869), .ZN(n1867) );
  AOI222_X1 U1928 ( .A1(opb_imag[11]), .A2(opa_imag[11]), .B1(opb_imag[11]), 
        .B2(n1867), .C1(opa_imag[11]), .C2(n1867), .ZN(n1865) );
  AOI222_X1 U1929 ( .A1(n1865), .A2(n1863), .B1(n1865), .B2(n1864), .C1(n1863), 
        .C2(n1864), .ZN(n1862) );
  AOI222_X1 U1930 ( .A1(opb_imag[13]), .A2(opa_imag[13]), .B1(opb_imag[13]), 
        .B2(n1862), .C1(opa_imag[13]), .C2(n1862), .ZN(n1860) );
  AOI222_X1 U1931 ( .A1(n1860), .A2(n1858), .B1(n1860), .B2(n1859), .C1(n1858), 
        .C2(n1859), .ZN(n1897) );
  XNOR2_X1 U1932 ( .A(n1901), .B(n1897), .ZN(N108) );
  AOI22_X1 U1933 ( .A1(opb_imag[14]), .A2(opa_imag[14]), .B1(n1859), .B2(n1858), .ZN(n1903) );
  XNOR2_X1 U1934 ( .A(n1860), .B(n1903), .ZN(N107) );
  XOR2_X1 U1935 ( .A(opb_imag[13]), .B(n1861), .Z(n1905) );
  XNOR2_X1 U1936 ( .A(n1905), .B(n1862), .ZN(N106) );
  AOI22_X1 U1937 ( .A1(opb_imag[12]), .A2(opa_imag[12]), .B1(n1864), .B2(n1863), .ZN(n1907) );
  XNOR2_X1 U1938 ( .A(n1865), .B(n1907), .ZN(N105) );
  XOR2_X1 U1939 ( .A(opb_imag[11]), .B(n1866), .Z(n1909) );
  XNOR2_X1 U1940 ( .A(n1909), .B(n1867), .ZN(N104) );
  OAI21_X1 U1941 ( .B1(opb_imag[10]), .B2(n1869), .A(n1868), .ZN(n1910) );
  XNOR2_X1 U1942 ( .A(n1870), .B(n1910), .ZN(N103) );
  AOI21_X1 U1943 ( .B1(opa_imag[9]), .B2(n1872), .A(n1871), .ZN(n1912) );
  XNOR2_X1 U1944 ( .A(n1912), .B(n1873), .ZN(N102) );
  OAI21_X1 U1945 ( .B1(opb_imag[8]), .B2(n1875), .A(n1874), .ZN(n1914) );
  XNOR2_X1 U1946 ( .A(n1876), .B(n1914), .ZN(N101) );
  AOI21_X1 U1947 ( .B1(opa_imag[7]), .B2(n1878), .A(n1877), .ZN(n1916) );
  XNOR2_X1 U1948 ( .A(n1916), .B(n1879), .ZN(N100) );
  OAI21_X1 U1949 ( .B1(opb_imag[6]), .B2(n1881), .A(n1880), .ZN(n1918) );
  XNOR2_X1 U1950 ( .A(n1882), .B(n1918), .ZN(N99) );
  AOI21_X1 U1951 ( .B1(opa_imag[5]), .B2(n1884), .A(n1883), .ZN(n1920) );
  XNOR2_X1 U1952 ( .A(n1920), .B(n1885), .ZN(N98) );
  OAI21_X1 U1953 ( .B1(opb_imag[4]), .B2(n1887), .A(n1886), .ZN(n1922) );
  XNOR2_X1 U1954 ( .A(n1888), .B(n1922), .ZN(N97) );
  AOI21_X1 U1955 ( .B1(opa_imag[3]), .B2(n1890), .A(n1889), .ZN(n1924) );
  XNOR2_X1 U1956 ( .A(n1924), .B(n1891), .ZN(N96) );
  OAI21_X1 U1957 ( .B1(opb_imag[2]), .B2(n1893), .A(n1892), .ZN(n1928) );
  XNOR2_X1 U1958 ( .A(n1894), .B(n1928), .ZN(N95) );
  XNOR2_X1 U1959 ( .A(n1896), .B(n1895), .ZN(N94) );
  AOI222_X1 U1960 ( .A1(n1899), .A2(n1898), .B1(n1899), .B2(n1897), .C1(n1898), 
        .C2(n1897), .ZN(N109) );
  XNOR2_X1 U1961 ( .A(n1901), .B(n1900), .ZN(xbuf_imag_stage1[14]) );
  XNOR2_X1 U1962 ( .A(n1903), .B(n1902), .ZN(xbuf_imag_stage1[13]) );
  XNOR2_X1 U1963 ( .A(n1905), .B(n1904), .ZN(xbuf_imag_stage1[12]) );
  XNOR2_X1 U1964 ( .A(n1907), .B(n1906), .ZN(xbuf_imag_stage1[11]) );
  XNOR2_X1 U1965 ( .A(n1909), .B(n1908), .ZN(xbuf_imag_stage1[10]) );
  XNOR2_X1 U1966 ( .A(n1911), .B(n1910), .ZN(xbuf_imag_stage1[9]) );
  XNOR2_X1 U1967 ( .A(n1913), .B(n1912), .ZN(xbuf_imag_stage1[8]) );
  XNOR2_X1 U1968 ( .A(n1915), .B(n1914), .ZN(xbuf_imag_stage1[7]) );
  XNOR2_X1 U1969 ( .A(n1917), .B(n1916), .ZN(xbuf_imag_stage1[6]) );
  XNOR2_X1 U1970 ( .A(n1919), .B(n1918), .ZN(xbuf_imag_stage1[5]) );
  XNOR2_X1 U1971 ( .A(n1921), .B(n1920), .ZN(xbuf_imag_stage1[4]) );
  XNOR2_X1 U1972 ( .A(n1923), .B(n1922), .ZN(xbuf_imag_stage1[3]) );
  XNOR2_X1 U1973 ( .A(n1925), .B(n1924), .ZN(xbuf_imag_stage1[2]) );
  XNOR2_X1 U1974 ( .A(n1927), .B(n1926), .ZN(xbuf_real_stage1[1]) );
  XNOR2_X1 U1975 ( .A(n1929), .B(n1928), .ZN(xbuf_imag_stage1[1]) );
  NOR2_X1 U1976 ( .A1(n1956), .A2(n1957), .ZN(\intadd_3/CI ) );
  XNOR2_X1 U1977 ( .A(n1931), .B(n1930), .ZN(xbuf_real_stage1[2]) );
  XNOR2_X1 U1978 ( .A(n1933), .B(n1932), .ZN(xbuf_real_stage1[3]) );
  XNOR2_X1 U1979 ( .A(n1935), .B(n1934), .ZN(xbuf_real_stage1[4]) );
  XNOR2_X1 U1980 ( .A(n1937), .B(n1936), .ZN(xbuf_real_stage1[5]) );
  XNOR2_X1 U1981 ( .A(n1939), .B(n1938), .ZN(xbuf_real_stage1[6]) );
  XNOR2_X1 U1982 ( .A(n1941), .B(n1940), .ZN(xbuf_real_stage1[7]) );
  XNOR2_X1 U1983 ( .A(n1943), .B(n1942), .ZN(xbuf_real_stage1[8]) );
  XNOR2_X1 U1984 ( .A(n1945), .B(n1944), .ZN(xbuf_real_stage1[9]) );
  XNOR2_X1 U1985 ( .A(n1947), .B(n1946), .ZN(xbuf_real_stage1[10]) );
  XNOR2_X1 U1986 ( .A(n1949), .B(n1948), .ZN(xbuf_real_stage1[11]) );
  XNOR2_X1 U1987 ( .A(n1951), .B(n1950), .ZN(xbuf_real_stage1[12]) );
  XNOR2_X1 U1988 ( .A(n1953), .B(n1952), .ZN(xbuf_real_stage1[13]) );
  XNOR2_X1 U1989 ( .A(n1955), .B(n1954), .ZN(xbuf_real_stage1[14]) );
  AOI21_X1 U1990 ( .B1(n1957), .B2(n1956), .A(\intadd_3/CI ), .ZN(
        xbuf_real_p_imag_stage1[0]) );
  OAI21_X1 U1991 ( .B1(n1959), .B2(tmp_a_stage3[30]), .A(n1958), .ZN(
        \intadd_1/B[14] ) );
  OAI21_X1 U1992 ( .B1(n1961), .B2(tmp_a_stage3[29]), .A(n1960), .ZN(
        \intadd_1/B[13] ) );
  OAI21_X1 U1993 ( .B1(n1963), .B2(tmp_a_stage3[28]), .A(n1962), .ZN(
        \intadd_1/B[12] ) );
  OAI21_X1 U1994 ( .B1(n1965), .B2(tmp_a_stage3[27]), .A(n1964), .ZN(
        \intadd_1/B[11] ) );
  OAI21_X1 U1995 ( .B1(n1967), .B2(tmp_a_stage3[26]), .A(n1966), .ZN(
        \intadd_1/B[10] ) );
  OAI21_X1 U1996 ( .B1(n1969), .B2(tmp_a_stage3[25]), .A(n1968), .ZN(
        \intadd_1/B[9] ) );
  OAI21_X1 U1997 ( .B1(n1971), .B2(tmp_a_stage3[24]), .A(n1970), .ZN(
        \intadd_1/B[8] ) );
  OAI21_X1 U1998 ( .B1(n1973), .B2(tmp_a_stage3[23]), .A(n1972), .ZN(
        \intadd_1/B[7] ) );
  OAI21_X1 U1999 ( .B1(n1975), .B2(tmp_a_stage3[22]), .A(n1974), .ZN(
        \intadd_1/B[6] ) );
  OAI21_X1 U2000 ( .B1(n1977), .B2(tmp_a_stage3[21]), .A(n1976), .ZN(
        \intadd_1/B[5] ) );
  OAI21_X1 U2001 ( .B1(n1979), .B2(tmp_a_stage3[20]), .A(n1978), .ZN(
        \intadd_1/B[4] ) );
  OAI21_X1 U2002 ( .B1(n1981), .B2(tmp_a_stage3[19]), .A(n1980), .ZN(
        \intadd_1/B[3] ) );
  OAI21_X1 U2003 ( .B1(n1983), .B2(tmp_a_stage3[18]), .A(n1982), .ZN(
        \intadd_1/B[2] ) );
  OAI21_X1 U2004 ( .B1(n1985), .B2(tmp_a_stage3[17]), .A(n1984), .ZN(
        \intadd_1/B[1] ) );
  NAND2_X1 U2005 ( .A1(tmp_a_stage3[14]), .A2(tmp_a_stage3[15]), .ZN(n1988) );
  INV_X1 U2006 ( .A(n1988), .ZN(n1987) );
  OAI21_X1 U2007 ( .B1(n1987), .B2(tmp_a_stage3[16]), .A(n1986), .ZN(
        \intadd_1/CI ) );
  OAI21_X1 U2008 ( .B1(tmp_a_stage3[14]), .B2(tmp_a_stage3[15]), .A(n1988), 
        .ZN(n1989) );
  INV_X1 U2009 ( .A(n1989), .ZN(n2038) );
  OAI21_X1 U2010 ( .B1(tmp_i_stage3[1]), .B2(n2046), .A(tmp_i_stage3[0]), .ZN(
        n1990) );
  OAI22_X1 U2011 ( .A1(tmp_a_stage3[0]), .A2(n1990), .B1(tmp_a_stage3[1]), 
        .B2(n2051), .ZN(n1991) );
  AOI222_X1 U2012 ( .A1(tmp_i_stage3[2]), .A2(n2048), .B1(tmp_i_stage3[2]), 
        .B2(n1991), .C1(n2048), .C2(n1991), .ZN(n1992) );
  AOI222_X1 U2013 ( .A1(tmp_a_stage3[3]), .A2(n2054), .B1(tmp_a_stage3[3]), 
        .B2(n1992), .C1(n2054), .C2(n1992), .ZN(n1993) );
  AOI222_X1 U2014 ( .A1(tmp_i_stage3[4]), .A2(n2052), .B1(tmp_i_stage3[4]), 
        .B2(n1993), .C1(n2052), .C2(n1993), .ZN(n1994) );
  AOI222_X1 U2015 ( .A1(tmp_a_stage3[5]), .A2(n2056), .B1(tmp_a_stage3[5]), 
        .B2(n1994), .C1(n2056), .C2(n1994), .ZN(n1995) );
  AOI222_X1 U2016 ( .A1(tmp_i_stage3[6]), .A2(n2055), .B1(tmp_i_stage3[6]), 
        .B2(n1995), .C1(n2055), .C2(n1995), .ZN(n1996) );
  AOI222_X1 U2017 ( .A1(tmp_a_stage3[7]), .A2(n2059), .B1(tmp_a_stage3[7]), 
        .B2(n1996), .C1(n2059), .C2(n1996), .ZN(n1997) );
  AOI222_X1 U2018 ( .A1(tmp_i_stage3[8]), .A2(n2058), .B1(tmp_i_stage3[8]), 
        .B2(n1997), .C1(n2058), .C2(n1997), .ZN(n1998) );
  AOI222_X1 U2019 ( .A1(tmp_a_stage3[9]), .A2(n2060), .B1(tmp_a_stage3[9]), 
        .B2(n1998), .C1(n2060), .C2(n1998), .ZN(n1999) );
  AOI222_X1 U2020 ( .A1(tmp_i_stage3[10]), .A2(n2061), .B1(tmp_i_stage3[10]), 
        .B2(n1999), .C1(n2061), .C2(n1999), .ZN(n2000) );
  AOI222_X1 U2021 ( .A1(tmp_a_stage3[11]), .A2(n2063), .B1(tmp_a_stage3[11]), 
        .B2(n2000), .C1(n2063), .C2(n2000), .ZN(n2001) );
  AOI222_X1 U2022 ( .A1(tmp_i_stage3[12]), .A2(n2064), .B1(tmp_i_stage3[12]), 
        .B2(n2001), .C1(n2064), .C2(n2001), .ZN(n2002) );
  AOI222_X1 U2023 ( .A1(tmp_a_stage3[13]), .A2(n2065), .B1(tmp_a_stage3[13]), 
        .B2(n2002), .C1(n2065), .C2(n2002), .ZN(n2003) );
  AOI222_X1 U2024 ( .A1(tmp_i_stage3[14]), .A2(tmp_a_stage3[14]), .B1(
        tmp_i_stage3[14]), .B2(n2003), .C1(tmp_a_stage3[14]), .C2(n2003), .ZN(
        n2031) );
  AOI222_X1 U2025 ( .A1(n2038), .A2(n2031), .B1(n2038), .B2(n2067), .C1(n2031), 
        .C2(n2067), .ZN(\intadd_2/CI ) );
  OAI21_X1 U2026 ( .B1(tmp_r_stage3[1]), .B2(n2046), .A(tmp_r_stage3[0]), .ZN(
        n2004) );
  OAI22_X1 U2027 ( .A1(tmp_a_stage3[0]), .A2(n2004), .B1(tmp_a_stage3[1]), 
        .B2(n2047), .ZN(n2005) );
  AOI222_X1 U2028 ( .A1(tmp_r_stage3[2]), .A2(n2048), .B1(tmp_r_stage3[2]), 
        .B2(n2005), .C1(n2048), .C2(n2005), .ZN(n2008) );
  NAND2_X1 U2029 ( .A1(tmp_a_stage3[3]), .A2(n2008), .ZN(n2006) );
  NAND2_X1 U2030 ( .A1(tmp_r_stage3[3]), .A2(n2006), .ZN(n2007) );
  OAI21_X1 U2031 ( .B1(tmp_a_stage3[3]), .B2(n2008), .A(n2007), .ZN(n2009) );
  AOI222_X1 U2032 ( .A1(tmp_r_stage3[4]), .A2(n2052), .B1(tmp_r_stage3[4]), 
        .B2(n2009), .C1(n2052), .C2(n2009), .ZN(n2012) );
  NAND2_X1 U2033 ( .A1(tmp_a_stage3[5]), .A2(n2012), .ZN(n2010) );
  NAND2_X1 U2034 ( .A1(tmp_r_stage3[5]), .A2(n2010), .ZN(n2011) );
  OAI21_X1 U2035 ( .B1(tmp_a_stage3[5]), .B2(n2012), .A(n2011), .ZN(n2013) );
  AOI222_X1 U2036 ( .A1(tmp_r_stage3[6]), .A2(n2055), .B1(tmp_r_stage3[6]), 
        .B2(n2013), .C1(n2055), .C2(n2013), .ZN(n2016) );
  NAND2_X1 U2037 ( .A1(tmp_a_stage3[7]), .A2(n2016), .ZN(n2014) );
  NAND2_X1 U2038 ( .A1(tmp_r_stage3[7]), .A2(n2014), .ZN(n2015) );
  OAI21_X1 U2039 ( .B1(tmp_a_stage3[7]), .B2(n2016), .A(n2015), .ZN(n2017) );
  AOI222_X1 U2040 ( .A1(tmp_r_stage3[8]), .A2(n2058), .B1(tmp_r_stage3[8]), 
        .B2(n2017), .C1(n2058), .C2(n2017), .ZN(n2020) );
  NAND2_X1 U2041 ( .A1(tmp_a_stage3[9]), .A2(n2020), .ZN(n2018) );
  NAND2_X1 U2042 ( .A1(tmp_r_stage3[9]), .A2(n2018), .ZN(n2019) );
  OAI21_X1 U2043 ( .B1(tmp_a_stage3[9]), .B2(n2020), .A(n2019), .ZN(n2021) );
  AOI222_X1 U2044 ( .A1(tmp_r_stage3[10]), .A2(n2061), .B1(tmp_r_stage3[10]), 
        .B2(n2021), .C1(n2061), .C2(n2021), .ZN(n2024) );
  NAND2_X1 U2045 ( .A1(tmp_a_stage3[11]), .A2(n2024), .ZN(n2022) );
  NAND2_X1 U2046 ( .A1(tmp_r_stage3[11]), .A2(n2022), .ZN(n2023) );
  OAI21_X1 U2047 ( .B1(tmp_a_stage3[11]), .B2(n2024), .A(n2023), .ZN(n2025) );
  AOI222_X1 U2048 ( .A1(tmp_r_stage3[12]), .A2(n2064), .B1(tmp_r_stage3[12]), 
        .B2(n2025), .C1(n2064), .C2(n2025), .ZN(n2028) );
  NAND2_X1 U2049 ( .A1(tmp_a_stage3[13]), .A2(n2028), .ZN(n2026) );
  NAND2_X1 U2050 ( .A1(tmp_r_stage3[13]), .A2(n2026), .ZN(n2027) );
  OAI21_X1 U2051 ( .B1(tmp_a_stage3[13]), .B2(n2028), .A(n2027), .ZN(n2029) );
  AOI222_X1 U2052 ( .A1(tmp_r_stage3[14]), .A2(tmp_a_stage3[14]), .B1(
        tmp_r_stage3[14]), .B2(n2029), .C1(tmp_a_stage3[14]), .C2(n2029), .ZN(
        n2036) );
  AOI222_X1 U2053 ( .A1(n2038), .A2(n2036), .B1(n2038), .B2(n2068), .C1(n2036), 
        .C2(n2068), .ZN(\intadd_1/B[0] ) );
  NOR2_X1 U2054 ( .A1(rst), .A2(n2070), .ZN(N112) );
  INV_X1 U2055 ( .A(iact), .ZN(n2030) );
  NOR2_X1 U2056 ( .A1(rst), .A2(n2030), .ZN(N111) );
  AND2_X1 U2057 ( .A1(\yi[31] ), .A2(\intadd_2/SUM[14] ), .ZN(n2035) );
  XOR2_X1 U2058 ( .A(n2067), .B(n2031), .Z(n2032) );
  XNOR2_X1 U2059 ( .A(n2038), .B(n2032), .ZN(n2033) );
  OR2_X1 U2060 ( .A1(\yi[31] ), .A2(\intadd_2/SUM[14] ), .ZN(n2034) );
  OAI21_X1 U2061 ( .B1(n2035), .B2(n2033), .A(n2034), .ZN(dst_opb_imag[0]) );
  OAI21_X1 U2062 ( .B1(n2035), .B2(\intadd_2/SUM[0] ), .A(n2034), .ZN(
        dst_opb_imag[1]) );
  OAI21_X1 U2063 ( .B1(n2035), .B2(\intadd_2/SUM[1] ), .A(n2034), .ZN(
        dst_opb_imag[2]) );
  OAI21_X1 U2064 ( .B1(n2035), .B2(\intadd_2/SUM[2] ), .A(n2034), .ZN(
        dst_opb_imag[3]) );
  OAI21_X1 U2065 ( .B1(n2035), .B2(\intadd_2/SUM[3] ), .A(n2034), .ZN(
        dst_opb_imag[4]) );
  OAI21_X1 U2066 ( .B1(n2035), .B2(\intadd_2/SUM[4] ), .A(n2034), .ZN(
        dst_opb_imag[5]) );
  OAI21_X1 U2067 ( .B1(n2035), .B2(\intadd_2/SUM[5] ), .A(n2034), .ZN(
        dst_opb_imag[6]) );
  OAI21_X1 U2068 ( .B1(n2035), .B2(\intadd_2/SUM[6] ), .A(n2034), .ZN(
        dst_opb_imag[7]) );
  OAI21_X1 U2069 ( .B1(n2035), .B2(\intadd_2/SUM[7] ), .A(n2034), .ZN(
        dst_opb_imag[8]) );
  OAI21_X1 U2070 ( .B1(n2035), .B2(\intadd_2/SUM[8] ), .A(n2034), .ZN(
        dst_opb_imag[9]) );
  OAI21_X1 U2071 ( .B1(n2035), .B2(\intadd_2/SUM[9] ), .A(n2034), .ZN(
        dst_opb_imag[10]) );
  OAI21_X1 U2072 ( .B1(n2035), .B2(\intadd_2/SUM[10] ), .A(n2034), .ZN(
        dst_opb_imag[11]) );
  OAI21_X1 U2073 ( .B1(n2035), .B2(\intadd_2/SUM[11] ), .A(n2034), .ZN(
        dst_opb_imag[12]) );
  OAI21_X1 U2074 ( .B1(n2035), .B2(\intadd_2/SUM[12] ), .A(n2034), .ZN(
        dst_opb_imag[13]) );
  OAI21_X1 U2075 ( .B1(n2035), .B2(\intadd_2/SUM[13] ), .A(n2034), .ZN(
        dst_opb_imag[14]) );
  AND2_X1 U2076 ( .A1(\yr[31] ), .A2(\intadd_1/SUM[14] ), .ZN(n2041) );
  XOR2_X1 U2077 ( .A(n2068), .B(n2036), .Z(n2037) );
  XNOR2_X1 U2078 ( .A(n2038), .B(n2037), .ZN(n2039) );
  OAI21_X1 U2079 ( .B1(n2041), .B2(n2039), .A(n2040), .ZN(dst_opb_real[0]) );
  OAI21_X1 U2080 ( .B1(n2041), .B2(\intadd_1/SUM[0] ), .A(n2040), .ZN(
        dst_opb_real[1]) );
  OAI21_X1 U2081 ( .B1(n2041), .B2(\intadd_1/SUM[1] ), .A(n2040), .ZN(
        dst_opb_real[2]) );
  OAI21_X1 U2082 ( .B1(n2041), .B2(\intadd_1/SUM[2] ), .A(n2040), .ZN(
        dst_opb_real[3]) );
  OAI21_X1 U2083 ( .B1(n2041), .B2(\intadd_1/SUM[3] ), .A(n2040), .ZN(
        dst_opb_real[4]) );
  OAI21_X1 U2084 ( .B1(n2041), .B2(\intadd_1/SUM[4] ), .A(n2040), .ZN(
        dst_opb_real[5]) );
  OAI21_X1 U2085 ( .B1(n2041), .B2(\intadd_1/SUM[5] ), .A(n2040), .ZN(
        dst_opb_real[6]) );
  OAI21_X1 U2086 ( .B1(n2041), .B2(\intadd_1/SUM[6] ), .A(n2040), .ZN(
        dst_opb_real[7]) );
  OAI21_X1 U2087 ( .B1(n2041), .B2(\intadd_1/SUM[7] ), .A(n2040), .ZN(
        dst_opb_real[8]) );
  OAI21_X1 U2088 ( .B1(n2041), .B2(\intadd_1/SUM[8] ), .A(n2040), .ZN(
        dst_opb_real[9]) );
  OAI21_X1 U2089 ( .B1(n2041), .B2(\intadd_1/SUM[9] ), .A(n2040), .ZN(
        dst_opb_real[10]) );
  OAI21_X1 U2090 ( .B1(n2041), .B2(\intadd_1/SUM[10] ), .A(n2040), .ZN(
        dst_opb_real[11]) );
  OAI21_X1 U2091 ( .B1(n2041), .B2(\intadd_1/SUM[11] ), .A(n2040), .ZN(
        dst_opb_real[12]) );
  OAI21_X1 U2092 ( .B1(n2041), .B2(\intadd_1/SUM[12] ), .A(n2040), .ZN(
        dst_opb_real[13]) );
  OAI21_X1 U2093 ( .B1(n2041), .B2(\intadd_1/SUM[13] ), .A(n2040), .ZN(
        dst_opb_real[14]) );
endmodule


module bfp_bitWidthDetector_FFT_BFPDW5_FFT_DW16_0 ( operand0, operand1, 
        operand2, operand3, bw );
  input [15:0] operand0;
  input [15:0] operand1;
  input [15:0] operand2;
  input [15:0] operand3;
  output [4:0] bw;
  wire   N51, N84, N117, N150, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11,
         n12, n13, n14, n15, n16, n17, n18, n19, n20, n21, n22, n23, n24, n25,
         n26, n27, n28, n29, n30, n31, n32, n33, n34, n35, n36, n37, n38, n39,
         n40, n41, n42, n43, n44, n45, n46, n47, n48, n49, n50, n51, n52, n53,
         n54, n55, n56, n57, n58, n59, n60, n61, n62, n63, n64, n65, n66, n67,
         n68, n69, n70, n71, n72, n73, n74, n75, n76, n77, n78, n79, n80, n81,
         n82, n83, n84, n85, n86, n87, n88, n89, n90, n91, n92, n93, n94, n95,
         n96, n97, n98, n99, n100, n101, n102, n103, n104, n105, n106, n107,
         n108, n109, n110, n111, n112, n113, n114, n115, n116, n117, n118,
         n119, n120, n121, n122, n123, n124, n125, n126, n127, n128, n129,
         n130, n131, n132, n133, n134, n135, n136, n137, n138, n139, n140,
         n141, n142, n143, n144, n145, n146, n147, n148, n149, n150, n151,
         n152, n153, n154, n155, n156, n157, n158, n159, n160, n161, n162,
         n163, n164, n165, n166, n167, n168, n169, n170, n171, n172, n173,
         n174, n175, n176, n177, n178, n179, n180, n181, n182, n183, n184,
         n185, n186, n187, n188, n189, n190, n191, n192, n193, n194, n195,
         n196, n197, n198, n199, n200, n201, n202, n203, n204, n205, n206,
         n207, n208, n209, n210, n211, n212, n213, n214, n215, n216, n217,
         n218, n219, n220;
  assign N51 = operand0[0];
  assign N84 = operand1[0];
  assign N117 = operand2[0];
  assign N150 = operand3[0];

  NOR3_X1 U3 ( .A1(operand2[2]), .A2(operand2[1]), .A3(N117), .ZN(n138) );
  INV_X1 U4 ( .A(operand2[3]), .ZN(n1) );
  NAND2_X1 U5 ( .A1(n138), .A2(n1), .ZN(n104) );
  NOR2_X1 U6 ( .A1(operand2[4]), .A2(n104), .ZN(n89) );
  INV_X1 U7 ( .A(operand2[5]), .ZN(n92) );
  NAND2_X1 U8 ( .A1(n89), .A2(n92), .ZN(n145) );
  NOR2_X1 U9 ( .A1(operand2[6]), .A2(n145), .ZN(n169) );
  INV_X1 U10 ( .A(operand2[7]), .ZN(n2) );
  NAND2_X1 U11 ( .A1(n169), .A2(n2), .ZN(n75) );
  NOR2_X1 U12 ( .A1(operand2[8]), .A2(n75), .ZN(n60) );
  INV_X1 U13 ( .A(operand2[9]), .ZN(n63) );
  NAND2_X1 U14 ( .A1(n60), .A2(n63), .ZN(n47) );
  NOR2_X1 U15 ( .A1(operand2[10]), .A2(n47), .ZN(n39) );
  INV_X1 U16 ( .A(operand2[11]), .ZN(n42) );
  NAND2_X1 U17 ( .A1(n39), .A2(n42), .ZN(n24) );
  NOR2_X1 U18 ( .A1(operand2[12]), .A2(n24), .ZN(n180) );
  INV_X1 U19 ( .A(operand2[13]), .ZN(n183) );
  NAND2_X1 U20 ( .A1(n180), .A2(n183), .ZN(n10) );
  INV_X1 U21 ( .A(operand2[15]), .ZN(n179) );
  NOR3_X1 U22 ( .A1(operand2[14]), .A2(n10), .A3(n179), .ZN(n6) );
  OR4_X1 U23 ( .A1(operand3[3]), .A2(operand3[2]), .A3(operand3[1]), .A4(N150), 
        .ZN(n105) );
  NOR2_X1 U24 ( .A1(operand3[4]), .A2(n105), .ZN(n88) );
  INV_X1 U25 ( .A(operand3[5]), .ZN(n94) );
  NAND2_X1 U26 ( .A1(n88), .A2(n94), .ZN(n150) );
  NOR2_X1 U27 ( .A1(operand3[6]), .A2(n150), .ZN(n162) );
  INV_X1 U28 ( .A(operand3[7]), .ZN(n168) );
  NAND2_X1 U29 ( .A1(n162), .A2(n168), .ZN(n76) );
  NOR2_X1 U30 ( .A1(operand3[8]), .A2(n76), .ZN(n59) );
  INV_X1 U31 ( .A(operand3[9]), .ZN(n65) );
  NAND2_X1 U32 ( .A1(n59), .A2(n65), .ZN(n53) );
  NOR2_X1 U33 ( .A1(operand3[10]), .A2(n53), .ZN(n38) );
  INV_X1 U34 ( .A(operand3[11]), .ZN(n44) );
  NAND2_X1 U35 ( .A1(n38), .A2(n44), .ZN(n25) );
  NOR2_X1 U36 ( .A1(operand3[12]), .A2(n25), .ZN(n178) );
  INV_X1 U37 ( .A(operand3[13]), .ZN(n185) );
  NAND2_X1 U38 ( .A1(n178), .A2(n185), .ZN(n7) );
  INV_X1 U39 ( .A(operand3[15]), .ZN(n177) );
  NOR3_X1 U40 ( .A1(operand3[14]), .A2(n7), .A3(n177), .ZN(n5) );
  OR4_X1 U41 ( .A1(operand0[3]), .A2(operand0[2]), .A3(operand0[1]), .A4(N51), 
        .ZN(n109) );
  NOR2_X1 U42 ( .A1(operand0[4]), .A2(n109), .ZN(n96) );
  INV_X1 U43 ( .A(operand0[5]), .ZN(n99) );
  NAND2_X1 U44 ( .A1(n96), .A2(n99), .ZN(n146) );
  NOR2_X1 U45 ( .A1(operand0[6]), .A2(n146), .ZN(n159) );
  INV_X1 U46 ( .A(operand0[7]), .ZN(n160) );
  NAND2_X1 U47 ( .A1(n159), .A2(n160), .ZN(n80) );
  NOR2_X1 U48 ( .A1(operand0[8]), .A2(n80), .ZN(n67) );
  INV_X1 U49 ( .A(operand0[9]), .ZN(n70) );
  NAND2_X1 U50 ( .A1(n67), .A2(n70), .ZN(n48) );
  NOR2_X1 U51 ( .A1(operand0[10]), .A2(n48), .ZN(n32) );
  INV_X1 U52 ( .A(operand0[11]), .ZN(n35) );
  NAND2_X1 U53 ( .A1(n32), .A2(n35), .ZN(n19) );
  NOR2_X1 U54 ( .A1(operand0[12]), .A2(n19), .ZN(n189) );
  INV_X1 U55 ( .A(operand0[13]), .ZN(n192) );
  NAND2_X1 U56 ( .A1(n189), .A2(n192), .ZN(n8) );
  INV_X1 U57 ( .A(operand0[15]), .ZN(n188) );
  NOR3_X1 U58 ( .A1(operand0[14]), .A2(n8), .A3(n188), .ZN(n4) );
  OR4_X1 U59 ( .A1(operand1[3]), .A2(operand1[2]), .A3(operand1[1]), .A4(N84), 
        .ZN(n110) );
  NOR2_X1 U60 ( .A1(operand1[4]), .A2(n110), .ZN(n95) );
  INV_X1 U61 ( .A(operand1[5]), .ZN(n101) );
  NAND2_X1 U62 ( .A1(n95), .A2(n101), .ZN(n151) );
  NOR2_X1 U63 ( .A1(operand1[6]), .A2(n151), .ZN(n163) );
  INV_X1 U64 ( .A(operand1[7]), .ZN(n165) );
  NAND2_X1 U65 ( .A1(n163), .A2(n165), .ZN(n81) );
  NOR2_X1 U66 ( .A1(operand1[8]), .A2(n81), .ZN(n66) );
  INV_X1 U67 ( .A(operand1[9]), .ZN(n72) );
  NAND2_X1 U68 ( .A1(n66), .A2(n72), .ZN(n49) );
  NOR2_X1 U69 ( .A1(operand1[10]), .A2(n49), .ZN(n31) );
  INV_X1 U70 ( .A(operand1[11]), .ZN(n37) );
  NAND2_X1 U71 ( .A1(n31), .A2(n37), .ZN(n20) );
  NOR2_X1 U72 ( .A1(operand1[12]), .A2(n20), .ZN(n187) );
  INV_X1 U73 ( .A(operand1[13]), .ZN(n194) );
  NAND2_X1 U74 ( .A1(n187), .A2(n194), .ZN(n11) );
  INV_X1 U75 ( .A(operand1[15]), .ZN(n186) );
  NOR3_X1 U76 ( .A1(operand1[14]), .A2(n11), .A3(n186), .ZN(n3) );
  OR4_X1 U77 ( .A1(n6), .A2(n5), .A3(n4), .A4(n3), .ZN(bw[4]) );
  NAND2_X1 U78 ( .A1(operand3[15]), .A2(n7), .ZN(n18) );
  NAND2_X1 U79 ( .A1(n8), .A2(operand0[15]), .ZN(n9) );
  XNOR2_X1 U80 ( .A(operand0[14]), .B(n9), .ZN(n17) );
  NAND2_X1 U81 ( .A1(operand2[15]), .A2(n10), .ZN(n14) );
  NAND2_X1 U82 ( .A1(operand1[15]), .A2(n11), .ZN(n13) );
  OAI22_X1 U83 ( .A1(operand2[14]), .A2(n14), .B1(n13), .B2(operand1[14]), 
        .ZN(n12) );
  AOI221_X1 U84 ( .B1(n14), .B2(operand2[14]), .C1(n13), .C2(operand1[14]), 
        .A(n12), .ZN(n15) );
  OAI21_X1 U85 ( .B1(n18), .B2(operand3[14]), .A(n15), .ZN(n16) );
  AOI211_X1 U86 ( .C1(n18), .C2(operand3[14]), .A(n17), .B(n16), .ZN(n206) );
  NAND2_X1 U87 ( .A1(operand0[15]), .A2(n19), .ZN(n23) );
  NAND2_X1 U88 ( .A1(operand1[15]), .A2(n20), .ZN(n22) );
  OAI22_X1 U89 ( .A1(operand0[12]), .A2(n23), .B1(n22), .B2(operand1[12]), 
        .ZN(n21) );
  AOI221_X1 U90 ( .B1(n23), .B2(operand0[12]), .C1(n22), .C2(operand1[12]), 
        .A(n21), .ZN(n30) );
  NAND2_X1 U91 ( .A1(operand2[15]), .A2(n24), .ZN(n28) );
  NAND2_X1 U92 ( .A1(operand3[15]), .A2(n25), .ZN(n27) );
  OAI22_X1 U93 ( .A1(operand2[12]), .A2(n28), .B1(n27), .B2(operand3[12]), 
        .ZN(n26) );
  AOI221_X1 U94 ( .B1(n28), .B2(operand2[12]), .C1(n27), .C2(operand3[12]), 
        .A(n26), .ZN(n29) );
  NAND2_X1 U95 ( .A1(n30), .A2(n29), .ZN(n198) );
  NOR2_X1 U96 ( .A1(n31), .A2(n186), .ZN(n36) );
  NOR2_X1 U97 ( .A1(n32), .A2(n188), .ZN(n34) );
  AOI22_X1 U98 ( .A1(n37), .A2(n36), .B1(n35), .B2(n34), .ZN(n33) );
  OAI221_X1 U99 ( .B1(n37), .B2(n36), .C1(n35), .C2(n34), .A(n33), .ZN(n46) );
  NOR2_X1 U100 ( .A1(n38), .A2(n177), .ZN(n43) );
  NOR2_X1 U101 ( .A1(n39), .A2(n179), .ZN(n41) );
  AOI22_X1 U102 ( .A1(n44), .A2(n43), .B1(n42), .B2(n41), .ZN(n40) );
  OAI221_X1 U103 ( .B1(n44), .B2(n43), .C1(n42), .C2(n41), .A(n40), .ZN(n45)
         );
  OR3_X1 U104 ( .A1(n198), .A2(n46), .A3(n45), .ZN(n211) );
  NAND2_X1 U105 ( .A1(operand2[15]), .A2(n47), .ZN(n58) );
  NAND2_X1 U106 ( .A1(operand0[15]), .A2(n48), .ZN(n52) );
  NAND2_X1 U107 ( .A1(operand1[15]), .A2(n49), .ZN(n51) );
  OAI22_X1 U108 ( .A1(operand0[10]), .A2(n52), .B1(n51), .B2(operand1[10]), 
        .ZN(n50) );
  AOI221_X1 U109 ( .B1(n52), .B2(operand0[10]), .C1(n51), .C2(operand1[10]), 
        .A(n50), .ZN(n57) );
  NAND2_X1 U110 ( .A1(n53), .A2(operand3[15]), .ZN(n54) );
  XNOR2_X1 U111 ( .A(operand3[10]), .B(n54), .ZN(n55) );
  AOI21_X1 U112 ( .B1(n58), .B2(operand2[10]), .A(n55), .ZN(n56) );
  OAI211_X1 U113 ( .C1(n58), .C2(operand2[10]), .A(n57), .B(n56), .ZN(n87) );
  NOR2_X1 U114 ( .A1(n59), .A2(n177), .ZN(n64) );
  NOR2_X1 U115 ( .A1(n60), .A2(n179), .ZN(n62) );
  AOI22_X1 U116 ( .A1(n65), .A2(n64), .B1(n63), .B2(n62), .ZN(n61) );
  OAI221_X1 U117 ( .B1(n65), .B2(n64), .C1(n63), .C2(n62), .A(n61), .ZN(n74)
         );
  NOR2_X1 U118 ( .A1(n66), .A2(n186), .ZN(n71) );
  NOR2_X1 U119 ( .A1(n67), .A2(n188), .ZN(n69) );
  AOI22_X1 U120 ( .A1(n72), .A2(n71), .B1(n70), .B2(n69), .ZN(n68) );
  OAI221_X1 U121 ( .B1(n72), .B2(n71), .C1(n70), .C2(n69), .A(n68), .ZN(n73)
         );
  OR3_X1 U122 ( .A1(n87), .A2(n74), .A3(n73), .ZN(n218) );
  NOR2_X1 U123 ( .A1(n211), .A2(n218), .ZN(n209) );
  NAND2_X1 U124 ( .A1(operand2[15]), .A2(n75), .ZN(n79) );
  NAND2_X1 U125 ( .A1(operand3[15]), .A2(n76), .ZN(n78) );
  OAI22_X1 U126 ( .A1(operand2[8]), .A2(n79), .B1(n78), .B2(operand3[8]), .ZN(
        n77) );
  AOI221_X1 U127 ( .B1(n79), .B2(operand2[8]), .C1(n78), .C2(operand3[8]), .A(
        n77), .ZN(n86) );
  NAND2_X1 U128 ( .A1(operand0[15]), .A2(n80), .ZN(n84) );
  NAND2_X1 U129 ( .A1(operand1[15]), .A2(n81), .ZN(n83) );
  OAI22_X1 U130 ( .A1(operand0[8]), .A2(n84), .B1(n83), .B2(operand1[8]), .ZN(
        n82) );
  AOI221_X1 U131 ( .B1(n84), .B2(operand0[8]), .C1(n83), .C2(operand1[8]), .A(
        n82), .ZN(n85) );
  NAND2_X1 U132 ( .A1(n86), .A2(n85), .ZN(n173) );
  AOI21_X1 U133 ( .B1(n209), .B2(n173), .A(n87), .ZN(n176) );
  NOR2_X1 U134 ( .A1(n88), .A2(n177), .ZN(n93) );
  NOR2_X1 U135 ( .A1(n89), .A2(n179), .ZN(n91) );
  AOI22_X1 U136 ( .A1(n94), .A2(n93), .B1(n92), .B2(n91), .ZN(n90) );
  OAI221_X1 U137 ( .B1(n94), .B2(n93), .C1(n92), .C2(n91), .A(n90), .ZN(n103)
         );
  NOR2_X1 U138 ( .A1(n95), .A2(n186), .ZN(n100) );
  NOR2_X1 U139 ( .A1(n96), .A2(n188), .ZN(n98) );
  AOI22_X1 U140 ( .A1(n101), .A2(n100), .B1(n99), .B2(n98), .ZN(n97) );
  OAI221_X1 U141 ( .B1(n101), .B2(n100), .C1(n99), .C2(n98), .A(n97), .ZN(n102) );
  NOR2_X1 U142 ( .A1(n103), .A2(n102), .ZN(n202) );
  NAND2_X1 U143 ( .A1(operand2[15]), .A2(n104), .ZN(n108) );
  NAND2_X1 U144 ( .A1(operand3[15]), .A2(n105), .ZN(n107) );
  OAI22_X1 U145 ( .A1(operand2[4]), .A2(n108), .B1(n107), .B2(operand3[4]), 
        .ZN(n106) );
  AOI221_X1 U146 ( .B1(n108), .B2(operand2[4]), .C1(n107), .C2(operand3[4]), 
        .A(n106), .ZN(n115) );
  NAND2_X1 U147 ( .A1(operand0[15]), .A2(n109), .ZN(n113) );
  NAND2_X1 U148 ( .A1(operand1[15]), .A2(n110), .ZN(n112) );
  OAI22_X1 U149 ( .A1(operand0[4]), .A2(n113), .B1(n112), .B2(operand1[4]), 
        .ZN(n111) );
  AOI221_X1 U150 ( .B1(n113), .B2(operand0[4]), .C1(n112), .C2(operand1[4]), 
        .A(n111), .ZN(n114) );
  NAND3_X1 U151 ( .A1(n202), .A2(n115), .A3(n114), .ZN(n158) );
  NOR4_X1 U152 ( .A1(N117), .A2(N150), .A3(N51), .A4(N84), .ZN(n144) );
  OAI21_X1 U153 ( .B1(operand2[1]), .B2(N117), .A(operand2[15]), .ZN(n123) );
  OAI21_X1 U154 ( .B1(operand0[1]), .B2(N51), .A(operand0[15]), .ZN(n116) );
  XNOR2_X1 U155 ( .A(operand0[2]), .B(n116), .ZN(n122) );
  OAI21_X1 U156 ( .B1(operand3[1]), .B2(N150), .A(operand3[15]), .ZN(n119) );
  OAI21_X1 U157 ( .B1(operand1[1]), .B2(N84), .A(operand1[15]), .ZN(n118) );
  OAI22_X1 U158 ( .A1(operand3[2]), .A2(n119), .B1(n118), .B2(operand1[2]), 
        .ZN(n117) );
  AOI221_X1 U159 ( .B1(n119), .B2(operand3[2]), .C1(n118), .C2(operand1[2]), 
        .A(n117), .ZN(n120) );
  OAI21_X1 U160 ( .B1(operand2[2]), .B2(n123), .A(n120), .ZN(n121) );
  AOI211_X1 U161 ( .C1(operand2[2]), .C2(n123), .A(n122), .B(n121), .ZN(n204)
         );
  NAND2_X1 U162 ( .A1(N117), .A2(operand2[15]), .ZN(n126) );
  NAND2_X1 U163 ( .A1(N150), .A2(operand3[15]), .ZN(n125) );
  OAI22_X1 U164 ( .A1(operand2[1]), .A2(n126), .B1(n125), .B2(operand3[1]), 
        .ZN(n124) );
  AOI221_X1 U165 ( .B1(n126), .B2(operand2[1]), .C1(n125), .C2(operand3[1]), 
        .A(n124), .ZN(n131) );
  NAND2_X1 U166 ( .A1(N51), .A2(operand0[15]), .ZN(n129) );
  NAND2_X1 U167 ( .A1(N84), .A2(operand1[15]), .ZN(n128) );
  OAI22_X1 U168 ( .A1(operand0[1]), .A2(n129), .B1(n128), .B2(operand1[1]), 
        .ZN(n127) );
  AOI221_X1 U169 ( .B1(n129), .B2(operand0[1]), .C1(n128), .C2(operand1[1]), 
        .A(n127), .ZN(n130) );
  NAND2_X1 U170 ( .A1(n131), .A2(n130), .ZN(n201) );
  OR3_X1 U171 ( .A1(operand0[2]), .A2(operand0[1]), .A3(N51), .ZN(n132) );
  NAND2_X1 U172 ( .A1(operand0[15]), .A2(n132), .ZN(n143) );
  OR3_X1 U173 ( .A1(operand3[2]), .A2(operand3[1]), .A3(N150), .ZN(n133) );
  NAND2_X1 U174 ( .A1(operand3[15]), .A2(n133), .ZN(n137) );
  OR3_X1 U175 ( .A1(operand1[2]), .A2(operand1[1]), .A3(N84), .ZN(n134) );
  NAND2_X1 U176 ( .A1(operand1[15]), .A2(n134), .ZN(n136) );
  OAI22_X1 U177 ( .A1(operand3[3]), .A2(n137), .B1(n136), .B2(operand1[3]), 
        .ZN(n135) );
  AOI221_X1 U178 ( .B1(n137), .B2(operand3[3]), .C1(n136), .C2(operand1[3]), 
        .A(n135), .ZN(n140) );
  OAI21_X1 U179 ( .B1(n179), .B2(n138), .A(operand2[3]), .ZN(n139) );
  OAI211_X1 U180 ( .C1(n179), .C2(operand2[3]), .A(n140), .B(n139), .ZN(n141)
         );
  AOI211_X1 U181 ( .C1(n143), .C2(operand0[3]), .A(n158), .B(n141), .ZN(n142)
         );
  OAI21_X1 U182 ( .B1(n143), .B2(operand0[3]), .A(n142), .ZN(n213) );
  AOI221_X1 U183 ( .B1(n144), .B2(n204), .C1(n201), .C2(n204), .A(n213), .ZN(
        n157) );
  NAND2_X1 U184 ( .A1(operand2[15]), .A2(n145), .ZN(n149) );
  NAND2_X1 U185 ( .A1(operand0[15]), .A2(n146), .ZN(n148) );
  OAI22_X1 U186 ( .A1(operand2[6]), .A2(n149), .B1(n148), .B2(operand0[6]), 
        .ZN(n147) );
  AOI221_X1 U187 ( .B1(n149), .B2(operand2[6]), .C1(n148), .C2(operand0[6]), 
        .A(n147), .ZN(n156) );
  NAND2_X1 U188 ( .A1(operand3[15]), .A2(n150), .ZN(n154) );
  NAND2_X1 U189 ( .A1(operand1[15]), .A2(n151), .ZN(n153) );
  OAI22_X1 U190 ( .A1(operand3[6]), .A2(n154), .B1(n153), .B2(operand1[6]), 
        .ZN(n152) );
  AOI221_X1 U191 ( .B1(n154), .B2(operand3[6]), .C1(n153), .C2(operand1[6]), 
        .A(n152), .ZN(n155) );
  NAND2_X1 U192 ( .A1(n156), .A2(n155), .ZN(n214) );
  AOI211_X1 U193 ( .C1(n202), .C2(n158), .A(n157), .B(n214), .ZN(n175) );
  NOR2_X1 U194 ( .A1(n188), .A2(n159), .ZN(n161) );
  XNOR2_X1 U195 ( .A(n161), .B(n160), .ZN(n174) );
  NOR2_X1 U196 ( .A1(n162), .A2(n177), .ZN(n167) );
  NOR2_X1 U197 ( .A1(n163), .A2(n186), .ZN(n166) );
  OAI22_X1 U198 ( .A1(n168), .A2(n167), .B1(n165), .B2(n166), .ZN(n164) );
  AOI221_X1 U199 ( .B1(n168), .B2(n167), .C1(n166), .C2(n165), .A(n164), .ZN(
        n171) );
  OAI21_X1 U200 ( .B1(n179), .B2(n169), .A(operand2[7]), .ZN(n170) );
  OAI211_X1 U201 ( .C1(n179), .C2(operand2[7]), .A(n171), .B(n170), .ZN(n172)
         );
  NOR3_X1 U202 ( .A1(n174), .A2(n173), .A3(n172), .ZN(n219) );
  NAND2_X1 U203 ( .A1(n209), .A2(n219), .ZN(n200) );
  OAI22_X1 U204 ( .A1(n176), .A2(n211), .B1(n175), .B2(n200), .ZN(n197) );
  NOR2_X1 U205 ( .A1(n178), .A2(n177), .ZN(n184) );
  NOR2_X1 U206 ( .A1(n180), .A2(n179), .ZN(n182) );
  AOI22_X1 U207 ( .A1(n185), .A2(n184), .B1(n183), .B2(n182), .ZN(n181) );
  OAI221_X1 U208 ( .B1(n185), .B2(n184), .C1(n183), .C2(n182), .A(n181), .ZN(
        n196) );
  NOR2_X1 U209 ( .A1(n187), .A2(n186), .ZN(n193) );
  NOR2_X1 U210 ( .A1(n189), .A2(n188), .ZN(n191) );
  AOI22_X1 U211 ( .A1(n194), .A2(n193), .B1(n192), .B2(n191), .ZN(n190) );
  OAI221_X1 U212 ( .B1(n194), .B2(n193), .C1(n192), .C2(n191), .A(n190), .ZN(
        n195) );
  NOR2_X1 U213 ( .A1(n196), .A2(n195), .ZN(n205) );
  OAI21_X1 U214 ( .B1(n198), .B2(n197), .A(n205), .ZN(n199) );
  AOI21_X1 U215 ( .B1(n206), .B2(n199), .A(bw[4]), .ZN(bw[0]) );
  INV_X1 U216 ( .A(n200), .ZN(n212) );
  INV_X1 U217 ( .A(n201), .ZN(n203) );
  OAI221_X1 U218 ( .B1(n213), .B2(n204), .C1(n213), .C2(n203), .A(n202), .ZN(
        n207) );
  NAND2_X1 U219 ( .A1(n206), .A2(n205), .ZN(n210) );
  AOI221_X1 U220 ( .B1(n214), .B2(n212), .C1(n207), .C2(n212), .A(n210), .ZN(
        n208) );
  AOI221_X1 U221 ( .B1(n209), .B2(n208), .C1(n211), .C2(n208), .A(bw[4]), .ZN(
        bw[1]) );
  NOR2_X1 U222 ( .A1(n211), .A2(n210), .ZN(n216) );
  OAI21_X1 U223 ( .B1(n214), .B2(n213), .A(n212), .ZN(n215) );
  AOI21_X1 U224 ( .B1(n216), .B2(n215), .A(bw[4]), .ZN(bw[2]) );
  INV_X1 U225 ( .A(n216), .ZN(n217) );
  NOR2_X1 U226 ( .A1(n218), .A2(n217), .ZN(n220) );
  AOI21_X1 U227 ( .B1(n220), .B2(n219), .A(bw[4]), .ZN(bw[3]) );
endmodule


module ramPipelineBridge_FFT_N10_FFT_DW16_0 ( clk, rst, iact, oact, ictrl, 
        octrl, iMemAddr, iEvenData, iOddData, oMemAddr, oEvenData, oOddData );
  input [1:0] ictrl;
  output [1:0] octrl;
  input [8:0] iMemAddr;
  input [31:0] iEvenData;
  input [31:0] iOddData;
  output [8:0] oMemAddr;
  output [31:0] oEvenData;
  output [31:0] oOddData;
  input clk, rst, iact;
  output oact;
  wire   actPipe, N7, N8, N9, N10, N11, N12, n1, n2, n3;
  wire   [8:0] memPipeAddr;
  wire   [31:0] evenPipeData;
  wire   [31:0] oddPipeData;
  wire   [1:0] ctrlPipe;
  wire   [31:0] ev0Data;
  wire   [31:0] od0Data;
  wire   [31:0] od1Data;
  tri   clk;
  tri   rst;

  DFF_X1 \memPipeAddr_reg[8]  ( .D(iMemAddr[8]), .CK(clk), .Q(memPipeAddr[8])
         );
  DFF_X1 \memPipeAddr_reg[7]  ( .D(iMemAddr[7]), .CK(clk), .Q(memPipeAddr[7])
         );
  DFF_X1 \memPipeAddr_reg[6]  ( .D(iMemAddr[6]), .CK(clk), .Q(memPipeAddr[6])
         );
  DFF_X1 \memPipeAddr_reg[5]  ( .D(iMemAddr[5]), .CK(clk), .Q(memPipeAddr[5])
         );
  DFF_X1 \memPipeAddr_reg[4]  ( .D(iMemAddr[4]), .CK(clk), .Q(memPipeAddr[4])
         );
  DFF_X1 \memPipeAddr_reg[3]  ( .D(iMemAddr[3]), .CK(clk), .Q(memPipeAddr[3])
         );
  DFF_X1 \memPipeAddr_reg[2]  ( .D(iMemAddr[2]), .CK(clk), .Q(memPipeAddr[2])
         );
  DFF_X1 \memPipeAddr_reg[1]  ( .D(iMemAddr[1]), .CK(clk), .Q(memPipeAddr[1])
         );
  DFF_X1 \memPipeAddr_reg[0]  ( .D(iMemAddr[0]), .CK(clk), .Q(memPipeAddr[0])
         );
  DFF_X1 \evenPipeData_reg[31]  ( .D(iEvenData[31]), .CK(clk), .Q(
        evenPipeData[31]) );
  DFF_X1 \evenPipeData_reg[30]  ( .D(iEvenData[30]), .CK(clk), .Q(
        evenPipeData[30]) );
  DFF_X1 \evenPipeData_reg[29]  ( .D(iEvenData[29]), .CK(clk), .Q(
        evenPipeData[29]) );
  DFF_X1 \evenPipeData_reg[28]  ( .D(iEvenData[28]), .CK(clk), .Q(
        evenPipeData[28]) );
  DFF_X1 \evenPipeData_reg[27]  ( .D(iEvenData[27]), .CK(clk), .Q(
        evenPipeData[27]) );
  DFF_X1 \evenPipeData_reg[26]  ( .D(iEvenData[26]), .CK(clk), .Q(
        evenPipeData[26]) );
  DFF_X1 \evenPipeData_reg[25]  ( .D(iEvenData[25]), .CK(clk), .Q(
        evenPipeData[25]) );
  DFF_X1 \evenPipeData_reg[24]  ( .D(iEvenData[24]), .CK(clk), .Q(
        evenPipeData[24]) );
  DFF_X1 \evenPipeData_reg[23]  ( .D(iEvenData[23]), .CK(clk), .Q(
        evenPipeData[23]) );
  DFF_X1 \evenPipeData_reg[22]  ( .D(iEvenData[22]), .CK(clk), .Q(
        evenPipeData[22]) );
  DFF_X1 \evenPipeData_reg[21]  ( .D(iEvenData[21]), .CK(clk), .Q(
        evenPipeData[21]) );
  DFF_X1 \evenPipeData_reg[20]  ( .D(iEvenData[20]), .CK(clk), .Q(
        evenPipeData[20]) );
  DFF_X1 \evenPipeData_reg[19]  ( .D(iEvenData[19]), .CK(clk), .Q(
        evenPipeData[19]) );
  DFF_X1 \evenPipeData_reg[18]  ( .D(iEvenData[18]), .CK(clk), .Q(
        evenPipeData[18]) );
  DFF_X1 \evenPipeData_reg[17]  ( .D(iEvenData[17]), .CK(clk), .Q(
        evenPipeData[17]) );
  DFF_X1 \evenPipeData_reg[16]  ( .D(iEvenData[16]), .CK(clk), .Q(
        evenPipeData[16]) );
  DFF_X1 \evenPipeData_reg[15]  ( .D(iEvenData[15]), .CK(clk), .Q(
        evenPipeData[15]) );
  DFF_X1 \evenPipeData_reg[14]  ( .D(iEvenData[14]), .CK(clk), .Q(
        evenPipeData[14]) );
  DFF_X1 \evenPipeData_reg[13]  ( .D(iEvenData[13]), .CK(clk), .Q(
        evenPipeData[13]) );
  DFF_X1 \evenPipeData_reg[12]  ( .D(iEvenData[12]), .CK(clk), .Q(
        evenPipeData[12]) );
  DFF_X1 \evenPipeData_reg[11]  ( .D(iEvenData[11]), .CK(clk), .Q(
        evenPipeData[11]) );
  DFF_X1 \evenPipeData_reg[10]  ( .D(iEvenData[10]), .CK(clk), .Q(
        evenPipeData[10]) );
  DFF_X1 \evenPipeData_reg[9]  ( .D(iEvenData[9]), .CK(clk), .Q(
        evenPipeData[9]) );
  DFF_X1 \evenPipeData_reg[8]  ( .D(iEvenData[8]), .CK(clk), .Q(
        evenPipeData[8]) );
  DFF_X1 \evenPipeData_reg[7]  ( .D(iEvenData[7]), .CK(clk), .Q(
        evenPipeData[7]) );
  DFF_X1 \evenPipeData_reg[6]  ( .D(iEvenData[6]), .CK(clk), .Q(
        evenPipeData[6]) );
  DFF_X1 \evenPipeData_reg[5]  ( .D(iEvenData[5]), .CK(clk), .Q(
        evenPipeData[5]) );
  DFF_X1 \evenPipeData_reg[4]  ( .D(iEvenData[4]), .CK(clk), .Q(
        evenPipeData[4]) );
  DFF_X1 \evenPipeData_reg[3]  ( .D(iEvenData[3]), .CK(clk), .Q(
        evenPipeData[3]) );
  DFF_X1 \evenPipeData_reg[2]  ( .D(iEvenData[2]), .CK(clk), .Q(
        evenPipeData[2]) );
  DFF_X1 \evenPipeData_reg[1]  ( .D(iEvenData[1]), .CK(clk), .Q(
        evenPipeData[1]) );
  DFF_X1 \evenPipeData_reg[0]  ( .D(iEvenData[0]), .CK(clk), .Q(
        evenPipeData[0]) );
  DFF_X1 \oddPipeData_reg[31]  ( .D(iOddData[31]), .CK(clk), .Q(
        oddPipeData[31]) );
  DFF_X1 \oddPipeData_reg[30]  ( .D(iOddData[30]), .CK(clk), .Q(
        oddPipeData[30]) );
  DFF_X1 \oddPipeData_reg[29]  ( .D(iOddData[29]), .CK(clk), .Q(
        oddPipeData[29]) );
  DFF_X1 \oddPipeData_reg[28]  ( .D(iOddData[28]), .CK(clk), .Q(
        oddPipeData[28]) );
  DFF_X1 \oddPipeData_reg[27]  ( .D(iOddData[27]), .CK(clk), .Q(
        oddPipeData[27]) );
  DFF_X1 \oddPipeData_reg[26]  ( .D(iOddData[26]), .CK(clk), .Q(
        oddPipeData[26]) );
  DFF_X1 \oddPipeData_reg[25]  ( .D(iOddData[25]), .CK(clk), .Q(
        oddPipeData[25]) );
  DFF_X1 \oddPipeData_reg[24]  ( .D(iOddData[24]), .CK(clk), .Q(
        oddPipeData[24]) );
  DFF_X1 \oddPipeData_reg[23]  ( .D(iOddData[23]), .CK(clk), .Q(
        oddPipeData[23]) );
  DFF_X1 \oddPipeData_reg[22]  ( .D(iOddData[22]), .CK(clk), .Q(
        oddPipeData[22]) );
  DFF_X1 \oddPipeData_reg[21]  ( .D(iOddData[21]), .CK(clk), .Q(
        oddPipeData[21]) );
  DFF_X1 \oddPipeData_reg[20]  ( .D(iOddData[20]), .CK(clk), .Q(
        oddPipeData[20]) );
  DFF_X1 \oddPipeData_reg[19]  ( .D(iOddData[19]), .CK(clk), .Q(
        oddPipeData[19]) );
  DFF_X1 \oddPipeData_reg[18]  ( .D(iOddData[18]), .CK(clk), .Q(
        oddPipeData[18]) );
  DFF_X1 \oddPipeData_reg[17]  ( .D(iOddData[17]), .CK(clk), .Q(
        oddPipeData[17]) );
  DFF_X1 \oddPipeData_reg[16]  ( .D(iOddData[16]), .CK(clk), .Q(
        oddPipeData[16]) );
  DFF_X1 \oddPipeData_reg[15]  ( .D(iOddData[15]), .CK(clk), .Q(
        oddPipeData[15]) );
  DFF_X1 \oddPipeData_reg[14]  ( .D(iOddData[14]), .CK(clk), .Q(
        oddPipeData[14]) );
  DFF_X1 \oddPipeData_reg[13]  ( .D(iOddData[13]), .CK(clk), .Q(
        oddPipeData[13]) );
  DFF_X1 \oddPipeData_reg[12]  ( .D(iOddData[12]), .CK(clk), .Q(
        oddPipeData[12]) );
  DFF_X1 \oddPipeData_reg[11]  ( .D(iOddData[11]), .CK(clk), .Q(
        oddPipeData[11]) );
  DFF_X1 \oddPipeData_reg[10]  ( .D(iOddData[10]), .CK(clk), .Q(
        oddPipeData[10]) );
  DFF_X1 \oddPipeData_reg[9]  ( .D(iOddData[9]), .CK(clk), .Q(oddPipeData[9])
         );
  DFF_X1 \oddPipeData_reg[8]  ( .D(iOddData[8]), .CK(clk), .Q(oddPipeData[8])
         );
  DFF_X1 \oddPipeData_reg[7]  ( .D(iOddData[7]), .CK(clk), .Q(oddPipeData[7])
         );
  DFF_X1 \oddPipeData_reg[6]  ( .D(iOddData[6]), .CK(clk), .Q(oddPipeData[6])
         );
  DFF_X1 \oddPipeData_reg[5]  ( .D(iOddData[5]), .CK(clk), .Q(oddPipeData[5])
         );
  DFF_X1 \oddPipeData_reg[4]  ( .D(iOddData[4]), .CK(clk), .Q(oddPipeData[4])
         );
  DFF_X1 \oddPipeData_reg[3]  ( .D(iOddData[3]), .CK(clk), .Q(oddPipeData[3])
         );
  DFF_X1 \oddPipeData_reg[2]  ( .D(iOddData[2]), .CK(clk), .Q(oddPipeData[2])
         );
  DFF_X1 \oddPipeData_reg[1]  ( .D(iOddData[1]), .CK(clk), .Q(oddPipeData[1])
         );
  DFF_X1 \oddPipeData_reg[0]  ( .D(iOddData[0]), .CK(clk), .Q(oddPipeData[0])
         );
  DFF_X1 \ctrlPipe_reg[1]  ( .D(N9), .CK(clk), .Q(ctrlPipe[1]) );
  DFF_X1 \ctrlPipe_reg[0]  ( .D(N8), .CK(clk), .Q(ctrlPipe[0]) );
  DFF_X1 actPipe_reg ( .D(N7), .CK(clk), .Q(actPipe) );
  DFF_X1 \ctrl_f_reg[1]  ( .D(N12), .CK(clk), .Q(octrl[1]) );
  DFF_X1 \ctrl_f_reg[0]  ( .D(N11), .CK(clk), .Q(octrl[0]) );
  DFF_X1 oact_f_reg ( .D(N10), .CK(clk), .Q(oact) );
  DFF_X1 \mem0Addr_reg[8]  ( .D(memPipeAddr[8]), .CK(clk), .Q(oMemAddr[8]) );
  DFF_X1 \mem0Addr_reg[7]  ( .D(memPipeAddr[7]), .CK(clk), .Q(oMemAddr[7]) );
  DFF_X1 \mem0Addr_reg[6]  ( .D(memPipeAddr[6]), .CK(clk), .Q(oMemAddr[6]) );
  DFF_X1 \mem0Addr_reg[5]  ( .D(memPipeAddr[5]), .CK(clk), .Q(oMemAddr[5]) );
  DFF_X1 \mem0Addr_reg[4]  ( .D(memPipeAddr[4]), .CK(clk), .Q(oMemAddr[4]) );
  DFF_X1 \mem0Addr_reg[3]  ( .D(memPipeAddr[3]), .CK(clk), .Q(oMemAddr[3]) );
  DFF_X1 \mem0Addr_reg[2]  ( .D(memPipeAddr[2]), .CK(clk), .Q(oMemAddr[2]) );
  DFF_X1 \mem0Addr_reg[1]  ( .D(memPipeAddr[1]), .CK(clk), .Q(oMemAddr[1]) );
  DFF_X1 \mem0Addr_reg[0]  ( .D(memPipeAddr[0]), .CK(clk), .Q(oMemAddr[0]) );
  DFF_X1 \ev0Data_reg[31]  ( .D(evenPipeData[31]), .CK(clk), .Q(ev0Data[31])
         );
  DFF_X1 \ev0Data_reg[30]  ( .D(evenPipeData[30]), .CK(clk), .Q(ev0Data[30])
         );
  DFF_X1 \ev0Data_reg[29]  ( .D(evenPipeData[29]), .CK(clk), .Q(ev0Data[29])
         );
  DFF_X1 \ev0Data_reg[28]  ( .D(evenPipeData[28]), .CK(clk), .Q(ev0Data[28])
         );
  DFF_X1 \ev0Data_reg[27]  ( .D(evenPipeData[27]), .CK(clk), .Q(ev0Data[27])
         );
  DFF_X1 \ev0Data_reg[26]  ( .D(evenPipeData[26]), .CK(clk), .Q(ev0Data[26])
         );
  DFF_X1 \ev0Data_reg[25]  ( .D(evenPipeData[25]), .CK(clk), .Q(ev0Data[25])
         );
  DFF_X1 \ev0Data_reg[24]  ( .D(evenPipeData[24]), .CK(clk), .Q(ev0Data[24])
         );
  DFF_X1 \ev0Data_reg[23]  ( .D(evenPipeData[23]), .CK(clk), .Q(ev0Data[23])
         );
  DFF_X1 \ev0Data_reg[22]  ( .D(evenPipeData[22]), .CK(clk), .Q(ev0Data[22])
         );
  DFF_X1 \ev0Data_reg[21]  ( .D(evenPipeData[21]), .CK(clk), .Q(ev0Data[21])
         );
  DFF_X1 \ev0Data_reg[20]  ( .D(evenPipeData[20]), .CK(clk), .Q(ev0Data[20])
         );
  DFF_X1 \ev0Data_reg[19]  ( .D(evenPipeData[19]), .CK(clk), .Q(ev0Data[19])
         );
  DFF_X1 \ev0Data_reg[18]  ( .D(evenPipeData[18]), .CK(clk), .Q(ev0Data[18])
         );
  DFF_X1 \ev0Data_reg[17]  ( .D(evenPipeData[17]), .CK(clk), .Q(ev0Data[17])
         );
  DFF_X1 \ev0Data_reg[16]  ( .D(evenPipeData[16]), .CK(clk), .Q(ev0Data[16])
         );
  DFF_X1 \ev0Data_reg[15]  ( .D(evenPipeData[15]), .CK(clk), .Q(ev0Data[15])
         );
  DFF_X1 \ev0Data_reg[14]  ( .D(evenPipeData[14]), .CK(clk), .Q(ev0Data[14])
         );
  DFF_X1 \ev0Data_reg[13]  ( .D(evenPipeData[13]), .CK(clk), .Q(ev0Data[13])
         );
  DFF_X1 \ev0Data_reg[12]  ( .D(evenPipeData[12]), .CK(clk), .Q(ev0Data[12])
         );
  DFF_X1 \ev0Data_reg[11]  ( .D(evenPipeData[11]), .CK(clk), .Q(ev0Data[11])
         );
  DFF_X1 \ev0Data_reg[10]  ( .D(evenPipeData[10]), .CK(clk), .Q(ev0Data[10])
         );
  DFF_X1 \ev0Data_reg[9]  ( .D(evenPipeData[9]), .CK(clk), .Q(ev0Data[9]) );
  DFF_X1 \ev0Data_reg[8]  ( .D(evenPipeData[8]), .CK(clk), .Q(ev0Data[8]) );
  DFF_X1 \ev0Data_reg[7]  ( .D(evenPipeData[7]), .CK(clk), .Q(ev0Data[7]) );
  DFF_X1 \ev0Data_reg[6]  ( .D(evenPipeData[6]), .CK(clk), .Q(ev0Data[6]) );
  DFF_X1 \ev0Data_reg[5]  ( .D(evenPipeData[5]), .CK(clk), .Q(ev0Data[5]) );
  DFF_X1 \ev0Data_reg[4]  ( .D(evenPipeData[4]), .CK(clk), .Q(ev0Data[4]) );
  DFF_X1 \ev0Data_reg[3]  ( .D(evenPipeData[3]), .CK(clk), .Q(ev0Data[3]) );
  DFF_X1 \ev0Data_reg[2]  ( .D(evenPipeData[2]), .CK(clk), .Q(ev0Data[2]) );
  DFF_X1 \ev0Data_reg[1]  ( .D(evenPipeData[1]), .CK(clk), .Q(ev0Data[1]) );
  DFF_X1 \ev0Data_reg[0]  ( .D(evenPipeData[0]), .CK(clk), .Q(ev0Data[0]) );
  DFF_X1 \od0Data_reg[31]  ( .D(oddPipeData[31]), .CK(clk), .Q(od0Data[31]) );
  DFF_X1 \od0Data_reg[30]  ( .D(oddPipeData[30]), .CK(clk), .Q(od0Data[30]) );
  DFF_X1 \od0Data_reg[29]  ( .D(oddPipeData[29]), .CK(clk), .Q(od0Data[29]) );
  DFF_X1 \od0Data_reg[28]  ( .D(oddPipeData[28]), .CK(clk), .Q(od0Data[28]) );
  DFF_X1 \od0Data_reg[27]  ( .D(oddPipeData[27]), .CK(clk), .Q(od0Data[27]) );
  DFF_X1 \od0Data_reg[26]  ( .D(oddPipeData[26]), .CK(clk), .Q(od0Data[26]) );
  DFF_X1 \od0Data_reg[25]  ( .D(oddPipeData[25]), .CK(clk), .Q(od0Data[25]) );
  DFF_X1 \od0Data_reg[24]  ( .D(oddPipeData[24]), .CK(clk), .Q(od0Data[24]) );
  DFF_X1 \od0Data_reg[23]  ( .D(oddPipeData[23]), .CK(clk), .Q(od0Data[23]) );
  DFF_X1 \od0Data_reg[22]  ( .D(oddPipeData[22]), .CK(clk), .Q(od0Data[22]) );
  DFF_X1 \od0Data_reg[21]  ( .D(oddPipeData[21]), .CK(clk), .Q(od0Data[21]) );
  DFF_X1 \od0Data_reg[20]  ( .D(oddPipeData[20]), .CK(clk), .Q(od0Data[20]) );
  DFF_X1 \od0Data_reg[19]  ( .D(oddPipeData[19]), .CK(clk), .Q(od0Data[19]) );
  DFF_X1 \od0Data_reg[18]  ( .D(oddPipeData[18]), .CK(clk), .Q(od0Data[18]) );
  DFF_X1 \od0Data_reg[17]  ( .D(oddPipeData[17]), .CK(clk), .Q(od0Data[17]) );
  DFF_X1 \od0Data_reg[16]  ( .D(oddPipeData[16]), .CK(clk), .Q(od0Data[16]) );
  DFF_X1 \od0Data_reg[15]  ( .D(oddPipeData[15]), .CK(clk), .Q(od0Data[15]) );
  DFF_X1 \od0Data_reg[14]  ( .D(oddPipeData[14]), .CK(clk), .Q(od0Data[14]) );
  DFF_X1 \od0Data_reg[13]  ( .D(oddPipeData[13]), .CK(clk), .Q(od0Data[13]) );
  DFF_X1 \od0Data_reg[12]  ( .D(oddPipeData[12]), .CK(clk), .Q(od0Data[12]) );
  DFF_X1 \od0Data_reg[11]  ( .D(oddPipeData[11]), .CK(clk), .Q(od0Data[11]) );
  DFF_X1 \od0Data_reg[10]  ( .D(oddPipeData[10]), .CK(clk), .Q(od0Data[10]) );
  DFF_X1 \od0Data_reg[9]  ( .D(oddPipeData[9]), .CK(clk), .Q(od0Data[9]) );
  DFF_X1 \od0Data_reg[8]  ( .D(oddPipeData[8]), .CK(clk), .Q(od0Data[8]) );
  DFF_X1 \od0Data_reg[7]  ( .D(oddPipeData[7]), .CK(clk), .Q(od0Data[7]) );
  DFF_X1 \od0Data_reg[6]  ( .D(oddPipeData[6]), .CK(clk), .Q(od0Data[6]) );
  DFF_X1 \od0Data_reg[5]  ( .D(oddPipeData[5]), .CK(clk), .Q(od0Data[5]) );
  DFF_X1 \od0Data_reg[4]  ( .D(oddPipeData[4]), .CK(clk), .Q(od0Data[4]) );
  DFF_X1 \od0Data_reg[3]  ( .D(oddPipeData[3]), .CK(clk), .Q(od0Data[3]) );
  DFF_X1 \od0Data_reg[2]  ( .D(oddPipeData[2]), .CK(clk), .Q(od0Data[2]) );
  DFF_X1 \od0Data_reg[1]  ( .D(oddPipeData[1]), .CK(clk), .Q(od0Data[1]) );
  DFF_X1 \od0Data_reg[0]  ( .D(oddPipeData[0]), .CK(clk), .Q(od0Data[0]) );
  DFF_X1 \od1Data_reg[31]  ( .D(od0Data[31]), .CK(clk), .Q(od1Data[31]) );
  DFF_X1 \od1Data_reg[30]  ( .D(od0Data[30]), .CK(clk), .Q(od1Data[30]) );
  DFF_X1 \od1Data_reg[29]  ( .D(od0Data[29]), .CK(clk), .Q(od1Data[29]) );
  DFF_X1 \od1Data_reg[28]  ( .D(od0Data[28]), .CK(clk), .Q(od1Data[28]) );
  DFF_X1 \od1Data_reg[27]  ( .D(od0Data[27]), .CK(clk), .Q(od1Data[27]) );
  DFF_X1 \od1Data_reg[26]  ( .D(od0Data[26]), .CK(clk), .Q(od1Data[26]) );
  DFF_X1 \od1Data_reg[25]  ( .D(od0Data[25]), .CK(clk), .Q(od1Data[25]) );
  DFF_X1 \od1Data_reg[24]  ( .D(od0Data[24]), .CK(clk), .Q(od1Data[24]) );
  DFF_X1 \od1Data_reg[23]  ( .D(od0Data[23]), .CK(clk), .Q(od1Data[23]) );
  DFF_X1 \od1Data_reg[22]  ( .D(od0Data[22]), .CK(clk), .Q(od1Data[22]) );
  DFF_X1 \od1Data_reg[21]  ( .D(od0Data[21]), .CK(clk), .Q(od1Data[21]) );
  DFF_X1 \od1Data_reg[20]  ( .D(od0Data[20]), .CK(clk), .Q(od1Data[20]) );
  DFF_X1 \od1Data_reg[19]  ( .D(od0Data[19]), .CK(clk), .Q(od1Data[19]) );
  DFF_X1 \od1Data_reg[18]  ( .D(od0Data[18]), .CK(clk), .Q(od1Data[18]) );
  DFF_X1 \od1Data_reg[17]  ( .D(od0Data[17]), .CK(clk), .Q(od1Data[17]) );
  DFF_X1 \od1Data_reg[16]  ( .D(od0Data[16]), .CK(clk), .Q(od1Data[16]) );
  DFF_X1 \od1Data_reg[15]  ( .D(od0Data[15]), .CK(clk), .Q(od1Data[15]) );
  DFF_X1 \od1Data_reg[14]  ( .D(od0Data[14]), .CK(clk), .Q(od1Data[14]) );
  DFF_X1 \od1Data_reg[13]  ( .D(od0Data[13]), .CK(clk), .Q(od1Data[13]) );
  DFF_X1 \od1Data_reg[12]  ( .D(od0Data[12]), .CK(clk), .Q(od1Data[12]) );
  DFF_X1 \od1Data_reg[11]  ( .D(od0Data[11]), .CK(clk), .Q(od1Data[11]) );
  DFF_X1 \od1Data_reg[10]  ( .D(od0Data[10]), .CK(clk), .Q(od1Data[10]) );
  DFF_X1 \od1Data_reg[9]  ( .D(od0Data[9]), .CK(clk), .Q(od1Data[9]) );
  DFF_X1 \od1Data_reg[8]  ( .D(od0Data[8]), .CK(clk), .Q(od1Data[8]) );
  DFF_X1 \od1Data_reg[7]  ( .D(od0Data[7]), .CK(clk), .Q(od1Data[7]) );
  DFF_X1 \od1Data_reg[6]  ( .D(od0Data[6]), .CK(clk), .Q(od1Data[6]) );
  DFF_X1 \od1Data_reg[5]  ( .D(od0Data[5]), .CK(clk), .Q(od1Data[5]) );
  DFF_X1 \od1Data_reg[4]  ( .D(od0Data[4]), .CK(clk), .Q(od1Data[4]) );
  DFF_X1 \od1Data_reg[3]  ( .D(od0Data[3]), .CK(clk), .Q(od1Data[3]) );
  DFF_X1 \od1Data_reg[2]  ( .D(od0Data[2]), .CK(clk), .Q(od1Data[2]) );
  DFF_X1 \od1Data_reg[1]  ( .D(od0Data[1]), .CK(clk), .Q(od1Data[1]) );
  DFF_X1 \od1Data_reg[0]  ( .D(od0Data[0]), .CK(clk), .Q(od1Data[0]) );
  CLKBUF_X1 U3 ( .A(octrl[1]), .Z(n2) );
  INV_X1 U4 ( .A(rst), .ZN(n1) );
  AND2_X1 U5 ( .A1(ctrlPipe[0]), .A2(n1), .ZN(N11) );
  AND2_X1 U6 ( .A1(ctrlPipe[1]), .A2(n1), .ZN(N12) );
  AND2_X1 U7 ( .A1(actPipe), .A2(n1), .ZN(N10) );
  AND2_X1 U8 ( .A1(ictrl[0]), .A2(n1), .ZN(N8) );
  AND2_X1 U9 ( .A1(ictrl[1]), .A2(n1), .ZN(N9) );
  AND2_X1 U10 ( .A1(iact), .A2(n1), .ZN(N7) );
  MUX2_X1 U11 ( .A(evenPipeData[0]), .B(od0Data[0]), .S(octrl[1]), .Z(
        oOddData[0]) );
  MUX2_X1 U12 ( .A(evenPipeData[1]), .B(od0Data[1]), .S(octrl[1]), .Z(
        oOddData[1]) );
  MUX2_X1 U13 ( .A(evenPipeData[2]), .B(od0Data[2]), .S(octrl[1]), .Z(
        oOddData[2]) );
  MUX2_X1 U14 ( .A(evenPipeData[3]), .B(od0Data[3]), .S(octrl[1]), .Z(
        oOddData[3]) );
  MUX2_X1 U15 ( .A(evenPipeData[4]), .B(od0Data[4]), .S(octrl[1]), .Z(
        oOddData[4]) );
  MUX2_X1 U16 ( .A(evenPipeData[5]), .B(od0Data[5]), .S(octrl[1]), .Z(
        oOddData[5]) );
  MUX2_X1 U17 ( .A(evenPipeData[6]), .B(od0Data[6]), .S(octrl[1]), .Z(
        oOddData[6]) );
  MUX2_X1 U18 ( .A(evenPipeData[7]), .B(od0Data[7]), .S(octrl[1]), .Z(
        oOddData[7]) );
  MUX2_X1 U19 ( .A(evenPipeData[8]), .B(od0Data[8]), .S(n2), .Z(oOddData[8])
         );
  MUX2_X1 U20 ( .A(evenPipeData[9]), .B(od0Data[9]), .S(n2), .Z(oOddData[9])
         );
  MUX2_X1 U21 ( .A(evenPipeData[10]), .B(od0Data[10]), .S(n2), .Z(oOddData[10]) );
  MUX2_X1 U22 ( .A(evenPipeData[11]), .B(od0Data[11]), .S(n2), .Z(oOddData[11]) );
  MUX2_X1 U23 ( .A(evenPipeData[12]), .B(od0Data[12]), .S(n2), .Z(oOddData[12]) );
  MUX2_X1 U24 ( .A(evenPipeData[13]), .B(od0Data[13]), .S(n2), .Z(oOddData[13]) );
  MUX2_X1 U25 ( .A(evenPipeData[14]), .B(od0Data[14]), .S(n2), .Z(oOddData[14]) );
  MUX2_X1 U26 ( .A(evenPipeData[15]), .B(od0Data[15]), .S(n2), .Z(oOddData[15]) );
  MUX2_X1 U27 ( .A(evenPipeData[16]), .B(od0Data[16]), .S(n2), .Z(oOddData[16]) );
  MUX2_X1 U28 ( .A(evenPipeData[17]), .B(od0Data[17]), .S(n2), .Z(oOddData[17]) );
  MUX2_X1 U29 ( .A(evenPipeData[18]), .B(od0Data[18]), .S(n2), .Z(oOddData[18]) );
  MUX2_X1 U30 ( .A(evenPipeData[19]), .B(od0Data[19]), .S(n2), .Z(oOddData[19]) );
  MUX2_X1 U31 ( .A(evenPipeData[20]), .B(od0Data[20]), .S(octrl[1]), .Z(
        oOddData[20]) );
  MUX2_X1 U32 ( .A(evenPipeData[21]), .B(od0Data[21]), .S(n2), .Z(oOddData[21]) );
  MUX2_X1 U33 ( .A(evenPipeData[22]), .B(od0Data[22]), .S(octrl[1]), .Z(
        oOddData[22]) );
  MUX2_X1 U34 ( .A(evenPipeData[23]), .B(od0Data[23]), .S(octrl[1]), .Z(
        oOddData[23]) );
  MUX2_X1 U35 ( .A(evenPipeData[24]), .B(od0Data[24]), .S(octrl[1]), .Z(
        oOddData[24]) );
  MUX2_X1 U36 ( .A(evenPipeData[25]), .B(od0Data[25]), .S(octrl[1]), .Z(
        oOddData[25]) );
  MUX2_X1 U37 ( .A(evenPipeData[26]), .B(od0Data[26]), .S(octrl[1]), .Z(
        oOddData[26]) );
  MUX2_X1 U38 ( .A(evenPipeData[27]), .B(od0Data[27]), .S(octrl[1]), .Z(
        oOddData[27]) );
  MUX2_X1 U39 ( .A(evenPipeData[28]), .B(od0Data[28]), .S(octrl[1]), .Z(
        oOddData[28]) );
  MUX2_X1 U40 ( .A(evenPipeData[29]), .B(od0Data[29]), .S(octrl[1]), .Z(
        oOddData[29]) );
  MUX2_X1 U41 ( .A(evenPipeData[30]), .B(od0Data[30]), .S(octrl[1]), .Z(
        oOddData[30]) );
  MUX2_X1 U42 ( .A(evenPipeData[31]), .B(od0Data[31]), .S(octrl[1]), .Z(
        oOddData[31]) );
  CLKBUF_X1 U43 ( .A(octrl[0]), .Z(n3) );
  MUX2_X1 U44 ( .A(ev0Data[0]), .B(od1Data[0]), .S(n3), .Z(oEvenData[0]) );
  MUX2_X1 U45 ( .A(ev0Data[1]), .B(od1Data[1]), .S(n3), .Z(oEvenData[1]) );
  MUX2_X1 U46 ( .A(ev0Data[2]), .B(od1Data[2]), .S(n3), .Z(oEvenData[2]) );
  MUX2_X1 U47 ( .A(ev0Data[3]), .B(od1Data[3]), .S(n3), .Z(oEvenData[3]) );
  MUX2_X1 U48 ( .A(ev0Data[4]), .B(od1Data[4]), .S(n3), .Z(oEvenData[4]) );
  MUX2_X1 U49 ( .A(ev0Data[5]), .B(od1Data[5]), .S(n3), .Z(oEvenData[5]) );
  MUX2_X1 U50 ( .A(ev0Data[6]), .B(od1Data[6]), .S(n3), .Z(oEvenData[6]) );
  MUX2_X1 U51 ( .A(ev0Data[7]), .B(od1Data[7]), .S(n3), .Z(oEvenData[7]) );
  MUX2_X1 U52 ( .A(ev0Data[8]), .B(od1Data[8]), .S(octrl[0]), .Z(oEvenData[8])
         );
  MUX2_X1 U53 ( .A(ev0Data[9]), .B(od1Data[9]), .S(octrl[0]), .Z(oEvenData[9])
         );
  MUX2_X1 U54 ( .A(ev0Data[10]), .B(od1Data[10]), .S(octrl[0]), .Z(
        oEvenData[10]) );
  MUX2_X1 U55 ( .A(ev0Data[11]), .B(od1Data[11]), .S(octrl[0]), .Z(
        oEvenData[11]) );
  MUX2_X1 U56 ( .A(ev0Data[12]), .B(od1Data[12]), .S(n3), .Z(oEvenData[12]) );
  MUX2_X1 U57 ( .A(ev0Data[13]), .B(od1Data[13]), .S(n3), .Z(oEvenData[13]) );
  MUX2_X1 U58 ( .A(ev0Data[14]), .B(od1Data[14]), .S(n3), .Z(oEvenData[14]) );
  MUX2_X1 U59 ( .A(ev0Data[15]), .B(od1Data[15]), .S(octrl[0]), .Z(
        oEvenData[15]) );
  MUX2_X1 U60 ( .A(ev0Data[16]), .B(od1Data[16]), .S(n3), .Z(oEvenData[16]) );
  MUX2_X1 U61 ( .A(ev0Data[17]), .B(od1Data[17]), .S(octrl[0]), .Z(
        oEvenData[17]) );
  MUX2_X1 U62 ( .A(ev0Data[18]), .B(od1Data[18]), .S(octrl[0]), .Z(
        oEvenData[18]) );
  MUX2_X1 U63 ( .A(ev0Data[19]), .B(od1Data[19]), .S(octrl[0]), .Z(
        oEvenData[19]) );
  MUX2_X1 U64 ( .A(ev0Data[20]), .B(od1Data[20]), .S(octrl[0]), .Z(
        oEvenData[20]) );
  MUX2_X1 U65 ( .A(ev0Data[21]), .B(od1Data[21]), .S(octrl[0]), .Z(
        oEvenData[21]) );
  MUX2_X1 U66 ( .A(ev0Data[22]), .B(od1Data[22]), .S(octrl[0]), .Z(
        oEvenData[22]) );
  MUX2_X1 U67 ( .A(ev0Data[23]), .B(od1Data[23]), .S(octrl[0]), .Z(
        oEvenData[23]) );
  MUX2_X1 U68 ( .A(ev0Data[24]), .B(od1Data[24]), .S(octrl[0]), .Z(
        oEvenData[24]) );
  MUX2_X1 U69 ( .A(ev0Data[25]), .B(od1Data[25]), .S(octrl[0]), .Z(
        oEvenData[25]) );
  MUX2_X1 U70 ( .A(ev0Data[26]), .B(od1Data[26]), .S(n3), .Z(oEvenData[26]) );
  MUX2_X1 U71 ( .A(ev0Data[27]), .B(od1Data[27]), .S(octrl[0]), .Z(
        oEvenData[27]) );
  MUX2_X1 U72 ( .A(ev0Data[28]), .B(od1Data[28]), .S(octrl[0]), .Z(
        oEvenData[28]) );
  MUX2_X1 U73 ( .A(ev0Data[29]), .B(od1Data[29]), .S(octrl[0]), .Z(
        oEvenData[29]) );
  MUX2_X1 U74 ( .A(ev0Data[30]), .B(od1Data[30]), .S(octrl[0]), .Z(
        oEvenData[30]) );
  MUX2_X1 U75 ( .A(ev0Data[31]), .B(od1Data[31]), .S(octrl[0]), .Z(
        oEvenData[31]) );
endmodule


module bfp_Shifter_FFT_DW16_FFT_BFPDW5_0 ( operand, bfp_operand, bw );
  input [15:0] operand;
  output [15:0] bfp_operand;
  input [4:0] bw;
  wire   N8, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15,
         n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29,
         n30, n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43,
         n44, n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57,
         n58, n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71,
         n72, n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85,
         n86, n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99,
         n100, n101;
  tri   [15:0] operand;
  assign N8 = bw[4];

  INV_X1 U2 ( .A(n95), .ZN(n92) );
  OR2_X1 U3 ( .A1(bw[0]), .A2(bw[1]), .ZN(n1) );
  INV_X1 U4 ( .A(n1), .ZN(n2) );
  AOI22_X2 U5 ( .A1(n101), .A2(n6), .B1(n2), .B2(n32), .ZN(n85) );
  CLKBUF_X1 U6 ( .A(bw[2]), .Z(n95) );
  NAND2_X1 U7 ( .A1(bw[0]), .A2(bw[1]), .ZN(n29) );
  INV_X1 U8 ( .A(operand[4]), .ZN(n9) );
  INV_X1 U9 ( .A(bw[1]), .ZN(n15) );
  NOR2_X1 U10 ( .A1(bw[0]), .A2(n15), .ZN(n67) );
  INV_X1 U11 ( .A(bw[0]), .ZN(n11) );
  NOR2_X1 U12 ( .A1(bw[1]), .A2(n11), .ZN(n74) );
  AOI22_X1 U13 ( .A1(n67), .A2(operand[3]), .B1(n74), .B2(operand[2]), .ZN(n3)
         );
  OAI21_X1 U14 ( .B1(n29), .B2(n9), .A(n3), .ZN(n4) );
  AOI21_X1 U15 ( .B1(n2), .B2(operand[1]), .A(n4), .ZN(n31) );
  INV_X1 U16 ( .A(operand[0]), .ZN(n18) );
  NOR2_X1 U17 ( .A1(n29), .A2(n18), .ZN(n34) );
  INV_X1 U18 ( .A(n34), .ZN(n5) );
  AOI22_X1 U19 ( .A1(n95), .A2(n31), .B1(n5), .B2(n92), .ZN(n64) );
  INV_X1 U20 ( .A(bw[3]), .ZN(n77) );
  INV_X1 U21 ( .A(n77), .ZN(n101) );
  NOR3_X1 U22 ( .A1(N8), .A2(n92), .A3(n29), .ZN(n6) );
  NOR2_X1 U23 ( .A1(n95), .A2(n101), .ZN(n32) );
  INV_X1 U24 ( .A(n85), .ZN(n96) );
  NOR2_X1 U25 ( .A1(N8), .A2(n96), .ZN(n89) );
  INV_X1 U26 ( .A(n89), .ZN(n83) );
  NOR2_X1 U27 ( .A1(n77), .A2(n83), .ZN(n97) );
  AOI22_X1 U28 ( .A1(n64), .A2(n97), .B1(n96), .B2(operand[4]), .ZN(n7) );
  INV_X1 U29 ( .A(n7), .ZN(bfp_operand[4]) );
  INV_X1 U30 ( .A(n74), .ZN(n90) );
  OAI22_X1 U31 ( .A1(operand[5]), .A2(n29), .B1(operand[3]), .B2(n90), .ZN(n8)
         );
  AOI21_X1 U32 ( .B1(n67), .B2(n9), .A(n8), .ZN(n10) );
  OAI21_X1 U33 ( .B1(operand[2]), .B2(n1), .A(n10), .ZN(n40) );
  INV_X1 U34 ( .A(operand[1]), .ZN(n19) );
  AOI22_X1 U35 ( .A1(bw[0]), .A2(n19), .B1(n18), .B2(n11), .ZN(n16) );
  NAND2_X1 U36 ( .A1(bw[1]), .A2(n16), .ZN(n36) );
  AOI22_X1 U37 ( .A1(n95), .A2(n40), .B1(n36), .B2(n92), .ZN(n72) );
  AOI22_X1 U38 ( .A1(n72), .A2(n97), .B1(n96), .B2(operand[5]), .ZN(n12) );
  INV_X1 U39 ( .A(n12), .ZN(bfp_operand[5]) );
  INV_X1 U40 ( .A(n67), .ZN(n91) );
  INV_X1 U41 ( .A(operand[6]), .ZN(n27) );
  INV_X1 U42 ( .A(n29), .ZN(n50) );
  AOI22_X1 U43 ( .A1(n50), .A2(operand[7]), .B1(n74), .B2(operand[5]), .ZN(n13) );
  OAI21_X1 U44 ( .B1(n91), .B2(n27), .A(n13), .ZN(n14) );
  AOI21_X1 U45 ( .B1(n2), .B2(operand[4]), .A(n14), .ZN(n54) );
  AOI222_X1 U46 ( .A1(n16), .A2(n15), .B1(n50), .B2(operand[3]), .C1(
        operand[2]), .C2(n67), .ZN(n49) );
  AOI22_X1 U47 ( .A1(n95), .A2(n54), .B1(n49), .B2(n92), .ZN(n88) );
  AOI22_X1 U48 ( .A1(n96), .A2(operand[7]), .B1(n88), .B2(n97), .ZN(n17) );
  INV_X1 U49 ( .A(n17), .ZN(bfp_operand[7]) );
  NOR2_X1 U50 ( .A1(n85), .A2(n18), .ZN(bfp_operand[0]) );
  NAND2_X1 U51 ( .A1(n95), .A2(n97), .ZN(n21) );
  OAI22_X1 U52 ( .A1(n85), .A2(n19), .B1(n21), .B2(n36), .ZN(bfp_operand[1])
         );
  INV_X1 U53 ( .A(operand[2]), .ZN(n20) );
  AOI222_X1 U54 ( .A1(n50), .A2(operand[2]), .B1(n67), .B2(operand[1]), .C1(
        n74), .C2(operand[0]), .ZN(n23) );
  OAI22_X1 U55 ( .A1(n85), .A2(n20), .B1(n23), .B2(n21), .ZN(bfp_operand[2])
         );
  INV_X1 U56 ( .A(operand[3]), .ZN(n22) );
  OAI22_X1 U57 ( .A1(n85), .A2(n22), .B1(n49), .B2(n21), .ZN(bfp_operand[3])
         );
  INV_X1 U58 ( .A(n23), .ZN(n81) );
  AOI22_X1 U59 ( .A1(n50), .A2(operand[6]), .B1(n67), .B2(operand[5]), .ZN(n25) );
  AOI22_X1 U60 ( .A1(n2), .A2(operand[3]), .B1(n74), .B2(operand[4]), .ZN(n24)
         );
  NAND2_X1 U61 ( .A1(n25), .A2(n24), .ZN(n80) );
  OAI221_X1 U62 ( .B1(n95), .B2(n81), .C1(n92), .C2(n80), .A(n97), .ZN(n26) );
  OAI21_X1 U63 ( .B1(n85), .B2(n27), .A(n26), .ZN(bfp_operand[6]) );
  INV_X1 U64 ( .A(operand[8]), .ZN(n38) );
  AOI22_X1 U65 ( .A1(n67), .A2(operand[7]), .B1(n74), .B2(operand[6]), .ZN(n28) );
  OAI21_X1 U66 ( .B1(n29), .B2(n38), .A(n28), .ZN(n30) );
  AOI21_X1 U67 ( .B1(n2), .B2(operand[5]), .A(n30), .ZN(n60) );
  AOI22_X1 U68 ( .A1(n95), .A2(n60), .B1(n31), .B2(n92), .ZN(n33) );
  NOR2_X1 U69 ( .A1(n32), .A2(n83), .ZN(n55) );
  OAI221_X1 U70 ( .B1(n101), .B2(n34), .C1(n77), .C2(n33), .A(n55), .ZN(n35)
         );
  OAI21_X1 U71 ( .B1(n85), .B2(n38), .A(n35), .ZN(bfp_operand[8]) );
  INV_X1 U72 ( .A(operand[9]), .ZN(n52) );
  INV_X1 U73 ( .A(n36), .ZN(n42) );
  AOI22_X1 U74 ( .A1(n50), .A2(operand[9]), .B1(n74), .B2(operand[7]), .ZN(n37) );
  OAI21_X1 U75 ( .B1(n91), .B2(n38), .A(n37), .ZN(n39) );
  AOI21_X1 U76 ( .B1(n2), .B2(operand[6]), .A(n39), .ZN(n68) );
  AOI22_X1 U77 ( .A1(n95), .A2(n68), .B1(n40), .B2(n92), .ZN(n41) );
  OAI221_X1 U78 ( .B1(n101), .B2(n42), .C1(n77), .C2(n41), .A(n55), .ZN(n43)
         );
  OAI21_X1 U79 ( .B1(n85), .B2(n52), .A(n43), .ZN(bfp_operand[9]) );
  AOI22_X1 U80 ( .A1(n50), .A2(operand[10]), .B1(n67), .B2(operand[9]), .ZN(
        n45) );
  AOI22_X1 U81 ( .A1(n2), .A2(operand[7]), .B1(n74), .B2(operand[8]), .ZN(n44)
         );
  NAND2_X1 U82 ( .A1(n45), .A2(n44), .ZN(n78) );
  AOI221_X1 U83 ( .B1(n95), .B2(n78), .C1(n92), .C2(n80), .A(n77), .ZN(n48) );
  OAI21_X1 U84 ( .B1(n101), .B2(n81), .A(n55), .ZN(n47) );
  INV_X1 U85 ( .A(operand[10]), .ZN(n46) );
  OAI22_X1 U86 ( .A1(n48), .A2(n47), .B1(n85), .B2(n46), .ZN(bfp_operand[10])
         );
  INV_X1 U87 ( .A(operand[11]), .ZN(n59) );
  INV_X1 U88 ( .A(n49), .ZN(n57) );
  AOI22_X1 U89 ( .A1(n50), .A2(operand[11]), .B1(n67), .B2(operand[10]), .ZN(
        n51) );
  OAI21_X1 U90 ( .B1(n52), .B2(n90), .A(n51), .ZN(n53) );
  AOI21_X1 U91 ( .B1(n2), .B2(operand[8]), .A(n53), .ZN(n93) );
  AOI22_X1 U92 ( .A1(n95), .A2(n93), .B1(n54), .B2(n92), .ZN(n56) );
  OAI221_X1 U93 ( .B1(n101), .B2(n57), .C1(n77), .C2(n56), .A(n55), .ZN(n58)
         );
  OAI21_X1 U94 ( .B1(n85), .B2(n59), .A(n58), .ZN(bfp_operand[11]) );
  INV_X1 U95 ( .A(operand[12]), .ZN(n66) );
  AOI21_X1 U96 ( .B1(n67), .B2(operand[11]), .A(n92), .ZN(n62) );
  AOI22_X1 U97 ( .A1(n2), .A2(operand[9]), .B1(n74), .B2(operand[10]), .ZN(n61) );
  AOI22_X1 U98 ( .A1(n62), .A2(n61), .B1(n60), .B2(n92), .ZN(n63) );
  OAI221_X1 U99 ( .B1(n101), .B2(n64), .C1(n77), .C2(n63), .A(n89), .ZN(n65)
         );
  OAI21_X1 U100 ( .B1(n85), .B2(n66), .A(n65), .ZN(bfp_operand[12]) );
  INV_X1 U101 ( .A(operand[13]), .ZN(n76) );
  AOI21_X1 U102 ( .B1(n67), .B2(operand[12]), .A(n92), .ZN(n70) );
  AOI22_X1 U103 ( .A1(n2), .A2(operand[10]), .B1(operand[11]), .B2(n74), .ZN(
        n69) );
  AOI22_X1 U104 ( .A1(n70), .A2(n69), .B1(n68), .B2(n92), .ZN(n71) );
  OAI221_X1 U105 ( .B1(n101), .B2(n72), .C1(n77), .C2(n71), .A(n89), .ZN(n73)
         );
  OAI21_X1 U106 ( .B1(n85), .B2(n76), .A(n73), .ZN(bfp_operand[13]) );
  AOI22_X1 U107 ( .A1(n2), .A2(operand[11]), .B1(n74), .B2(operand[12]), .ZN(
        n75) );
  OAI21_X1 U108 ( .B1(n91), .B2(n76), .A(n75), .ZN(n79) );
  AOI221_X1 U109 ( .B1(n95), .B2(n79), .C1(n92), .C2(n78), .A(n77), .ZN(n87)
         );
  AOI221_X1 U110 ( .B1(n92), .B2(n81), .C1(n95), .C2(n80), .A(n101), .ZN(n82)
         );
  OR2_X1 U111 ( .A1(n83), .A2(n82), .ZN(n86) );
  INV_X1 U112 ( .A(operand[14]), .ZN(n84) );
  OAI22_X1 U113 ( .A1(n87), .A2(n86), .B1(n85), .B2(n84), .ZN(bfp_operand[14])
         );
  NAND2_X1 U114 ( .A1(n89), .A2(n88), .ZN(n100) );
  OAI222_X1 U115 ( .A1(n91), .A2(operand[14]), .B1(n90), .B2(operand[13]), 
        .C1(n1), .C2(operand[12]), .ZN(n94) );
  AOI22_X1 U116 ( .A1(n95), .A2(n94), .B1(n93), .B2(n92), .ZN(n98) );
  AOI22_X1 U117 ( .A1(n98), .A2(n97), .B1(operand[15]), .B2(n96), .ZN(n99) );
  OAI21_X1 U118 ( .B1(n101), .B2(n100), .A(n99), .ZN(bfp_operand[15]) );
endmodule


module bfp_Shifter_FFT_DW16_FFT_BFPDW5_1 ( operand, bfp_operand, bw );
  input [15:0] operand;
  output [15:0] bfp_operand;
  input [4:0] bw;
  wire   N8, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15,
         n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29,
         n30, n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43,
         n44, n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57,
         n58, n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71,
         n72, n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85,
         n86, n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99,
         n100, n101;
  tri   [15:0] operand;
  assign N8 = bw[4];

  INV_X1 U2 ( .A(n95), .ZN(n92) );
  NOR2_X2 U3 ( .A1(bw[0]), .A2(bw[1]), .ZN(n73) );
  AOI22_X2 U4 ( .A1(n101), .A2(n4), .B1(n73), .B2(n30), .ZN(n84) );
  CLKBUF_X1 U5 ( .A(bw[2]), .Z(n95) );
  INV_X1 U6 ( .A(bw[3]), .ZN(n76) );
  INV_X1 U7 ( .A(n73), .ZN(n89) );
  INV_X1 U8 ( .A(bw[1]), .ZN(n13) );
  NOR2_X1 U9 ( .A1(bw[0]), .A2(n13), .ZN(n65) );
  INV_X1 U10 ( .A(operand[4]), .ZN(n7) );
  NAND2_X1 U11 ( .A1(bw[0]), .A2(bw[1]), .ZN(n27) );
  INV_X1 U12 ( .A(bw[0]), .ZN(n3) );
  NOR2_X1 U13 ( .A1(bw[1]), .A2(n3), .ZN(n72) );
  INV_X1 U14 ( .A(n72), .ZN(n90) );
  OAI22_X1 U15 ( .A1(operand[5]), .A2(n27), .B1(operand[3]), .B2(n90), .ZN(n1)
         );
  AOI21_X1 U16 ( .B1(n65), .B2(n7), .A(n1), .ZN(n2) );
  OAI21_X1 U17 ( .B1(operand[2]), .B2(n89), .A(n2), .ZN(n38) );
  INV_X1 U18 ( .A(operand[1]), .ZN(n17) );
  INV_X1 U19 ( .A(operand[0]), .ZN(n16) );
  AOI22_X1 U20 ( .A1(bw[0]), .A2(n17), .B1(n16), .B2(n3), .ZN(n14) );
  NAND2_X1 U21 ( .A1(bw[1]), .A2(n14), .ZN(n34) );
  AOI22_X1 U22 ( .A1(n95), .A2(n38), .B1(n34), .B2(n92), .ZN(n70) );
  INV_X1 U23 ( .A(n76), .ZN(n101) );
  NOR3_X1 U24 ( .A1(N8), .A2(n92), .A3(n27), .ZN(n4) );
  NOR2_X1 U25 ( .A1(n95), .A2(n101), .ZN(n30) );
  INV_X1 U26 ( .A(n84), .ZN(n96) );
  NOR2_X1 U27 ( .A1(N8), .A2(n96), .ZN(n88) );
  INV_X1 U28 ( .A(n88), .ZN(n82) );
  NOR2_X1 U29 ( .A1(n76), .A2(n82), .ZN(n97) );
  AOI22_X1 U30 ( .A1(n70), .A2(n97), .B1(n96), .B2(operand[5]), .ZN(n5) );
  INV_X1 U31 ( .A(n5), .ZN(bfp_operand[5]) );
  AOI22_X1 U32 ( .A1(n65), .A2(operand[3]), .B1(n72), .B2(operand[2]), .ZN(n6)
         );
  OAI21_X1 U33 ( .B1(n27), .B2(n7), .A(n6), .ZN(n8) );
  AOI21_X1 U34 ( .B1(n73), .B2(operand[1]), .A(n8), .ZN(n29) );
  NOR2_X1 U35 ( .A1(n27), .A2(n16), .ZN(n32) );
  INV_X1 U36 ( .A(n32), .ZN(n9) );
  AOI22_X1 U37 ( .A1(n95), .A2(n29), .B1(n9), .B2(n92), .ZN(n62) );
  AOI22_X1 U38 ( .A1(n62), .A2(n97), .B1(n96), .B2(operand[4]), .ZN(n10) );
  INV_X1 U39 ( .A(n10), .ZN(bfp_operand[4]) );
  INV_X1 U40 ( .A(n65), .ZN(n91) );
  INV_X1 U41 ( .A(operand[6]), .ZN(n25) );
  INV_X1 U42 ( .A(n27), .ZN(n48) );
  AOI22_X1 U43 ( .A1(n48), .A2(operand[7]), .B1(n72), .B2(operand[5]), .ZN(n11) );
  OAI21_X1 U44 ( .B1(n91), .B2(n25), .A(n11), .ZN(n12) );
  AOI21_X1 U45 ( .B1(n73), .B2(operand[4]), .A(n12), .ZN(n52) );
  AOI222_X1 U46 ( .A1(n14), .A2(n13), .B1(n48), .B2(operand[3]), .C1(
        operand[2]), .C2(n65), .ZN(n47) );
  AOI22_X1 U47 ( .A1(n95), .A2(n52), .B1(n47), .B2(n92), .ZN(n87) );
  AOI22_X1 U48 ( .A1(n96), .A2(operand[7]), .B1(n87), .B2(n97), .ZN(n15) );
  INV_X1 U49 ( .A(n15), .ZN(bfp_operand[7]) );
  NOR2_X1 U50 ( .A1(n84), .A2(n16), .ZN(bfp_operand[0]) );
  NAND2_X1 U51 ( .A1(n95), .A2(n97), .ZN(n19) );
  OAI22_X1 U52 ( .A1(n84), .A2(n17), .B1(n19), .B2(n34), .ZN(bfp_operand[1])
         );
  INV_X1 U53 ( .A(operand[2]), .ZN(n18) );
  AOI222_X1 U54 ( .A1(n48), .A2(operand[2]), .B1(n65), .B2(operand[1]), .C1(
        n72), .C2(operand[0]), .ZN(n21) );
  OAI22_X1 U55 ( .A1(n84), .A2(n18), .B1(n21), .B2(n19), .ZN(bfp_operand[2])
         );
  INV_X1 U56 ( .A(operand[3]), .ZN(n20) );
  OAI22_X1 U57 ( .A1(n84), .A2(n20), .B1(n47), .B2(n19), .ZN(bfp_operand[3])
         );
  INV_X1 U58 ( .A(n21), .ZN(n80) );
  AOI22_X1 U59 ( .A1(n48), .A2(operand[6]), .B1(n65), .B2(operand[5]), .ZN(n23) );
  AOI22_X1 U60 ( .A1(n73), .A2(operand[3]), .B1(n72), .B2(operand[4]), .ZN(n22) );
  NAND2_X1 U61 ( .A1(n23), .A2(n22), .ZN(n79) );
  OAI221_X1 U62 ( .B1(n95), .B2(n80), .C1(n92), .C2(n79), .A(n97), .ZN(n24) );
  OAI21_X1 U63 ( .B1(n84), .B2(n25), .A(n24), .ZN(bfp_operand[6]) );
  INV_X1 U64 ( .A(operand[8]), .ZN(n36) );
  AOI22_X1 U65 ( .A1(n65), .A2(operand[7]), .B1(n72), .B2(operand[6]), .ZN(n26) );
  OAI21_X1 U66 ( .B1(n27), .B2(n36), .A(n26), .ZN(n28) );
  AOI21_X1 U67 ( .B1(n73), .B2(operand[5]), .A(n28), .ZN(n58) );
  AOI22_X1 U68 ( .A1(n95), .A2(n58), .B1(n29), .B2(n92), .ZN(n31) );
  NOR2_X1 U69 ( .A1(n30), .A2(n82), .ZN(n53) );
  OAI221_X1 U70 ( .B1(n101), .B2(n32), .C1(n76), .C2(n31), .A(n53), .ZN(n33)
         );
  OAI21_X1 U71 ( .B1(n84), .B2(n36), .A(n33), .ZN(bfp_operand[8]) );
  INV_X1 U72 ( .A(operand[9]), .ZN(n50) );
  INV_X1 U73 ( .A(n34), .ZN(n40) );
  AOI22_X1 U74 ( .A1(n48), .A2(operand[9]), .B1(n72), .B2(operand[7]), .ZN(n35) );
  OAI21_X1 U75 ( .B1(n91), .B2(n36), .A(n35), .ZN(n37) );
  AOI21_X1 U76 ( .B1(n73), .B2(operand[6]), .A(n37), .ZN(n66) );
  AOI22_X1 U77 ( .A1(n95), .A2(n66), .B1(n38), .B2(n92), .ZN(n39) );
  OAI221_X1 U78 ( .B1(n101), .B2(n40), .C1(n76), .C2(n39), .A(n53), .ZN(n41)
         );
  OAI21_X1 U79 ( .B1(n84), .B2(n50), .A(n41), .ZN(bfp_operand[9]) );
  AOI22_X1 U80 ( .A1(n48), .A2(operand[10]), .B1(n65), .B2(operand[9]), .ZN(
        n43) );
  AOI22_X1 U81 ( .A1(n73), .A2(operand[7]), .B1(n72), .B2(operand[8]), .ZN(n42) );
  NAND2_X1 U82 ( .A1(n43), .A2(n42), .ZN(n77) );
  AOI221_X1 U83 ( .B1(n95), .B2(n77), .C1(n92), .C2(n79), .A(n76), .ZN(n46) );
  OAI21_X1 U84 ( .B1(n101), .B2(n80), .A(n53), .ZN(n45) );
  INV_X1 U85 ( .A(operand[10]), .ZN(n44) );
  OAI22_X1 U86 ( .A1(n46), .A2(n45), .B1(n84), .B2(n44), .ZN(bfp_operand[10])
         );
  INV_X1 U87 ( .A(operand[11]), .ZN(n57) );
  INV_X1 U88 ( .A(n47), .ZN(n55) );
  AOI22_X1 U89 ( .A1(n48), .A2(operand[11]), .B1(n65), .B2(operand[10]), .ZN(
        n49) );
  OAI21_X1 U90 ( .B1(n50), .B2(n90), .A(n49), .ZN(n51) );
  AOI21_X1 U91 ( .B1(n73), .B2(operand[8]), .A(n51), .ZN(n93) );
  AOI22_X1 U92 ( .A1(n95), .A2(n93), .B1(n52), .B2(n92), .ZN(n54) );
  OAI221_X1 U93 ( .B1(n101), .B2(n55), .C1(n76), .C2(n54), .A(n53), .ZN(n56)
         );
  OAI21_X1 U94 ( .B1(n84), .B2(n57), .A(n56), .ZN(bfp_operand[11]) );
  INV_X1 U95 ( .A(operand[12]), .ZN(n64) );
  AOI21_X1 U96 ( .B1(n65), .B2(operand[11]), .A(n92), .ZN(n60) );
  AOI22_X1 U97 ( .A1(n73), .A2(operand[9]), .B1(n72), .B2(operand[10]), .ZN(
        n59) );
  AOI22_X1 U98 ( .A1(n60), .A2(n59), .B1(n58), .B2(n92), .ZN(n61) );
  OAI221_X1 U99 ( .B1(n101), .B2(n62), .C1(n76), .C2(n61), .A(n88), .ZN(n63)
         );
  OAI21_X1 U100 ( .B1(n84), .B2(n64), .A(n63), .ZN(bfp_operand[12]) );
  INV_X1 U101 ( .A(operand[13]), .ZN(n75) );
  AOI21_X1 U102 ( .B1(n65), .B2(operand[12]), .A(n92), .ZN(n68) );
  AOI22_X1 U103 ( .A1(n73), .A2(operand[10]), .B1(operand[11]), .B2(n72), .ZN(
        n67) );
  AOI22_X1 U104 ( .A1(n68), .A2(n67), .B1(n66), .B2(n92), .ZN(n69) );
  OAI221_X1 U105 ( .B1(n101), .B2(n70), .C1(n76), .C2(n69), .A(n88), .ZN(n71)
         );
  OAI21_X1 U106 ( .B1(n84), .B2(n75), .A(n71), .ZN(bfp_operand[13]) );
  AOI22_X1 U107 ( .A1(n73), .A2(operand[11]), .B1(n72), .B2(operand[12]), .ZN(
        n74) );
  OAI21_X1 U108 ( .B1(n91), .B2(n75), .A(n74), .ZN(n78) );
  AOI221_X1 U109 ( .B1(n95), .B2(n78), .C1(n92), .C2(n77), .A(n76), .ZN(n86)
         );
  AOI221_X1 U110 ( .B1(n92), .B2(n80), .C1(n95), .C2(n79), .A(n101), .ZN(n81)
         );
  OR2_X1 U111 ( .A1(n82), .A2(n81), .ZN(n85) );
  INV_X1 U112 ( .A(operand[14]), .ZN(n83) );
  OAI22_X1 U113 ( .A1(n86), .A2(n85), .B1(n84), .B2(n83), .ZN(bfp_operand[14])
         );
  NAND2_X1 U114 ( .A1(n88), .A2(n87), .ZN(n100) );
  OAI222_X1 U115 ( .A1(n91), .A2(operand[14]), .B1(n90), .B2(operand[13]), 
        .C1(n89), .C2(operand[12]), .ZN(n94) );
  AOI22_X1 U116 ( .A1(n95), .A2(n94), .B1(n93), .B2(n92), .ZN(n98) );
  AOI22_X1 U117 ( .A1(n98), .A2(n97), .B1(operand[15]), .B2(n96), .ZN(n99) );
  OAI21_X1 U118 ( .B1(n101), .B2(n100), .A(n99), .ZN(bfp_operand[15]) );
endmodule


module bfp_Shifter_FFT_DW16_FFT_BFPDW5_2 ( operand, bfp_operand, bw );
  input [15:0] operand;
  output [15:0] bfp_operand;
  input [4:0] bw;
  wire   N8, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15,
         n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29,
         n30, n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43,
         n44, n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57,
         n58, n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71,
         n72, n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85,
         n86, n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99,
         n100;
  tri   [15:0] operand;
  assign N8 = bw[4];

  INV_X1 U2 ( .A(bw[2]), .ZN(n92) );
  AOI22_X2 U3 ( .A1(n100), .A2(n4), .B1(n73), .B2(n30), .ZN(n84) );
  NOR2_X2 U4 ( .A1(bw[0]), .A2(bw[1]), .ZN(n73) );
  INV_X1 U5 ( .A(bw[3]), .ZN(n76) );
  INV_X1 U6 ( .A(n73), .ZN(n89) );
  INV_X1 U7 ( .A(bw[1]), .ZN(n13) );
  NOR2_X1 U8 ( .A1(bw[0]), .A2(n13), .ZN(n65) );
  INV_X1 U9 ( .A(operand[4]), .ZN(n7) );
  NAND2_X1 U10 ( .A1(bw[0]), .A2(bw[1]), .ZN(n27) );
  INV_X1 U11 ( .A(bw[0]), .ZN(n3) );
  NOR2_X1 U12 ( .A1(bw[1]), .A2(n3), .ZN(n72) );
  INV_X1 U13 ( .A(n72), .ZN(n90) );
  OAI22_X1 U14 ( .A1(operand[5]), .A2(n27), .B1(operand[3]), .B2(n90), .ZN(n1)
         );
  AOI21_X1 U15 ( .B1(n65), .B2(n7), .A(n1), .ZN(n2) );
  OAI21_X1 U16 ( .B1(operand[2]), .B2(n89), .A(n2), .ZN(n38) );
  INV_X1 U17 ( .A(operand[1]), .ZN(n17) );
  INV_X1 U18 ( .A(operand[0]), .ZN(n16) );
  AOI22_X1 U19 ( .A1(bw[0]), .A2(n17), .B1(n16), .B2(n3), .ZN(n14) );
  NAND2_X1 U20 ( .A1(bw[1]), .A2(n14), .ZN(n34) );
  AOI22_X1 U21 ( .A1(bw[2]), .A2(n38), .B1(n34), .B2(n92), .ZN(n70) );
  INV_X1 U22 ( .A(n76), .ZN(n100) );
  NOR3_X1 U23 ( .A1(N8), .A2(n92), .A3(n27), .ZN(n4) );
  NOR2_X1 U24 ( .A1(bw[2]), .A2(n100), .ZN(n30) );
  INV_X1 U25 ( .A(n84), .ZN(n95) );
  NOR2_X1 U26 ( .A1(N8), .A2(n95), .ZN(n88) );
  INV_X1 U27 ( .A(n88), .ZN(n82) );
  NOR2_X1 U28 ( .A1(n76), .A2(n82), .ZN(n96) );
  AOI22_X1 U29 ( .A1(n70), .A2(n96), .B1(n95), .B2(operand[5]), .ZN(n5) );
  INV_X1 U30 ( .A(n5), .ZN(bfp_operand[5]) );
  AOI22_X1 U31 ( .A1(n65), .A2(operand[3]), .B1(n72), .B2(operand[2]), .ZN(n6)
         );
  OAI21_X1 U32 ( .B1(n27), .B2(n7), .A(n6), .ZN(n8) );
  AOI21_X1 U33 ( .B1(n73), .B2(operand[1]), .A(n8), .ZN(n29) );
  NOR2_X1 U34 ( .A1(n27), .A2(n16), .ZN(n32) );
  INV_X1 U35 ( .A(n32), .ZN(n9) );
  AOI22_X1 U36 ( .A1(bw[2]), .A2(n29), .B1(n9), .B2(n92), .ZN(n62) );
  AOI22_X1 U37 ( .A1(n62), .A2(n96), .B1(n95), .B2(operand[4]), .ZN(n10) );
  INV_X1 U38 ( .A(n10), .ZN(bfp_operand[4]) );
  INV_X1 U39 ( .A(n65), .ZN(n91) );
  INV_X1 U40 ( .A(operand[6]), .ZN(n25) );
  INV_X1 U41 ( .A(n27), .ZN(n48) );
  AOI22_X1 U42 ( .A1(n48), .A2(operand[7]), .B1(n72), .B2(operand[5]), .ZN(n11) );
  OAI21_X1 U43 ( .B1(n91), .B2(n25), .A(n11), .ZN(n12) );
  AOI21_X1 U44 ( .B1(n73), .B2(operand[4]), .A(n12), .ZN(n52) );
  AOI222_X1 U45 ( .A1(n14), .A2(n13), .B1(n48), .B2(operand[3]), .C1(
        operand[2]), .C2(n65), .ZN(n47) );
  AOI22_X1 U46 ( .A1(bw[2]), .A2(n52), .B1(n47), .B2(n92), .ZN(n87) );
  AOI22_X1 U47 ( .A1(n95), .A2(operand[7]), .B1(n87), .B2(n96), .ZN(n15) );
  INV_X1 U48 ( .A(n15), .ZN(bfp_operand[7]) );
  NOR2_X1 U49 ( .A1(n84), .A2(n16), .ZN(bfp_operand[0]) );
  NAND2_X1 U50 ( .A1(bw[2]), .A2(n96), .ZN(n19) );
  OAI22_X1 U51 ( .A1(n84), .A2(n17), .B1(n19), .B2(n34), .ZN(bfp_operand[1])
         );
  INV_X1 U52 ( .A(operand[2]), .ZN(n18) );
  AOI222_X1 U53 ( .A1(n48), .A2(operand[2]), .B1(n65), .B2(operand[1]), .C1(
        n72), .C2(operand[0]), .ZN(n21) );
  OAI22_X1 U54 ( .A1(n84), .A2(n18), .B1(n21), .B2(n19), .ZN(bfp_operand[2])
         );
  INV_X1 U55 ( .A(operand[3]), .ZN(n20) );
  OAI22_X1 U56 ( .A1(n84), .A2(n20), .B1(n47), .B2(n19), .ZN(bfp_operand[3])
         );
  INV_X1 U57 ( .A(n21), .ZN(n80) );
  AOI22_X1 U58 ( .A1(n48), .A2(operand[6]), .B1(n65), .B2(operand[5]), .ZN(n23) );
  AOI22_X1 U59 ( .A1(n73), .A2(operand[3]), .B1(n72), .B2(operand[4]), .ZN(n22) );
  NAND2_X1 U60 ( .A1(n23), .A2(n22), .ZN(n79) );
  OAI221_X1 U61 ( .B1(bw[2]), .B2(n80), .C1(n92), .C2(n79), .A(n96), .ZN(n24)
         );
  OAI21_X1 U62 ( .B1(n84), .B2(n25), .A(n24), .ZN(bfp_operand[6]) );
  INV_X1 U63 ( .A(operand[8]), .ZN(n36) );
  AOI22_X1 U64 ( .A1(n65), .A2(operand[7]), .B1(n72), .B2(operand[6]), .ZN(n26) );
  OAI21_X1 U65 ( .B1(n27), .B2(n36), .A(n26), .ZN(n28) );
  AOI21_X1 U66 ( .B1(n73), .B2(operand[5]), .A(n28), .ZN(n58) );
  AOI22_X1 U67 ( .A1(bw[2]), .A2(n58), .B1(n29), .B2(n92), .ZN(n31) );
  NOR2_X1 U68 ( .A1(n30), .A2(n82), .ZN(n53) );
  OAI221_X1 U69 ( .B1(n100), .B2(n32), .C1(n76), .C2(n31), .A(n53), .ZN(n33)
         );
  OAI21_X1 U70 ( .B1(n84), .B2(n36), .A(n33), .ZN(bfp_operand[8]) );
  INV_X1 U71 ( .A(operand[9]), .ZN(n50) );
  INV_X1 U72 ( .A(n34), .ZN(n40) );
  AOI22_X1 U73 ( .A1(n48), .A2(operand[9]), .B1(n72), .B2(operand[7]), .ZN(n35) );
  OAI21_X1 U74 ( .B1(n91), .B2(n36), .A(n35), .ZN(n37) );
  AOI21_X1 U75 ( .B1(n73), .B2(operand[6]), .A(n37), .ZN(n66) );
  AOI22_X1 U76 ( .A1(bw[2]), .A2(n66), .B1(n38), .B2(n92), .ZN(n39) );
  OAI221_X1 U77 ( .B1(n100), .B2(n40), .C1(n76), .C2(n39), .A(n53), .ZN(n41)
         );
  OAI21_X1 U78 ( .B1(n84), .B2(n50), .A(n41), .ZN(bfp_operand[9]) );
  AOI22_X1 U79 ( .A1(n48), .A2(operand[10]), .B1(n65), .B2(operand[9]), .ZN(
        n43) );
  AOI22_X1 U80 ( .A1(n73), .A2(operand[7]), .B1(n72), .B2(operand[8]), .ZN(n42) );
  NAND2_X1 U81 ( .A1(n43), .A2(n42), .ZN(n77) );
  AOI221_X1 U82 ( .B1(bw[2]), .B2(n77), .C1(n92), .C2(n79), .A(n76), .ZN(n46)
         );
  OAI21_X1 U83 ( .B1(n100), .B2(n80), .A(n53), .ZN(n45) );
  INV_X1 U84 ( .A(operand[10]), .ZN(n44) );
  OAI22_X1 U85 ( .A1(n46), .A2(n45), .B1(n84), .B2(n44), .ZN(bfp_operand[10])
         );
  INV_X1 U86 ( .A(operand[11]), .ZN(n57) );
  INV_X1 U87 ( .A(n47), .ZN(n55) );
  AOI22_X1 U88 ( .A1(n48), .A2(operand[11]), .B1(n65), .B2(operand[10]), .ZN(
        n49) );
  OAI21_X1 U89 ( .B1(n50), .B2(n90), .A(n49), .ZN(n51) );
  AOI21_X1 U90 ( .B1(n73), .B2(operand[8]), .A(n51), .ZN(n93) );
  AOI22_X1 U91 ( .A1(bw[2]), .A2(n93), .B1(n52), .B2(n92), .ZN(n54) );
  OAI221_X1 U92 ( .B1(n100), .B2(n55), .C1(n76), .C2(n54), .A(n53), .ZN(n56)
         );
  OAI21_X1 U93 ( .B1(n84), .B2(n57), .A(n56), .ZN(bfp_operand[11]) );
  INV_X1 U94 ( .A(operand[12]), .ZN(n64) );
  AOI21_X1 U95 ( .B1(n65), .B2(operand[11]), .A(n92), .ZN(n60) );
  AOI22_X1 U96 ( .A1(n73), .A2(operand[9]), .B1(n72), .B2(operand[10]), .ZN(
        n59) );
  AOI22_X1 U97 ( .A1(n60), .A2(n59), .B1(n58), .B2(n92), .ZN(n61) );
  OAI221_X1 U98 ( .B1(n100), .B2(n62), .C1(n76), .C2(n61), .A(n88), .ZN(n63)
         );
  OAI21_X1 U99 ( .B1(n84), .B2(n64), .A(n63), .ZN(bfp_operand[12]) );
  INV_X1 U100 ( .A(operand[13]), .ZN(n75) );
  AOI21_X1 U101 ( .B1(n65), .B2(operand[12]), .A(n92), .ZN(n68) );
  AOI22_X1 U102 ( .A1(n73), .A2(operand[10]), .B1(operand[11]), .B2(n72), .ZN(
        n67) );
  AOI22_X1 U103 ( .A1(n68), .A2(n67), .B1(n66), .B2(n92), .ZN(n69) );
  OAI221_X1 U104 ( .B1(n100), .B2(n70), .C1(n76), .C2(n69), .A(n88), .ZN(n71)
         );
  OAI21_X1 U105 ( .B1(n84), .B2(n75), .A(n71), .ZN(bfp_operand[13]) );
  AOI22_X1 U106 ( .A1(n73), .A2(operand[11]), .B1(n72), .B2(operand[12]), .ZN(
        n74) );
  OAI21_X1 U107 ( .B1(n91), .B2(n75), .A(n74), .ZN(n78) );
  AOI221_X1 U108 ( .B1(bw[2]), .B2(n78), .C1(n92), .C2(n77), .A(n76), .ZN(n86)
         );
  AOI221_X1 U109 ( .B1(n92), .B2(n80), .C1(bw[2]), .C2(n79), .A(n100), .ZN(n81) );
  OR2_X1 U110 ( .A1(n82), .A2(n81), .ZN(n85) );
  INV_X1 U111 ( .A(operand[14]), .ZN(n83) );
  OAI22_X1 U112 ( .A1(n86), .A2(n85), .B1(n84), .B2(n83), .ZN(bfp_operand[14])
         );
  NAND2_X1 U113 ( .A1(n88), .A2(n87), .ZN(n99) );
  OAI222_X1 U114 ( .A1(n91), .A2(operand[14]), .B1(n90), .B2(operand[13]), 
        .C1(n89), .C2(operand[12]), .ZN(n94) );
  AOI22_X1 U115 ( .A1(bw[2]), .A2(n94), .B1(n93), .B2(n92), .ZN(n97) );
  AOI22_X1 U116 ( .A1(n97), .A2(n96), .B1(operand[15]), .B2(n95), .ZN(n98) );
  OAI21_X1 U117 ( .B1(n100), .B2(n99), .A(n98), .ZN(bfp_operand[15]) );
endmodule


module butterflyCore_FFT_N10_FFT_DW16_FFT_BFPDW5_PL_DEPTH3 ( clk, rst, clr_bfp, 
        ibfp, bw_ramwrite, iact, oact, ictrl, octrl, iMemAddr, iEvenData, 
        iOddData, oMemAddr, oEvenData, oOddData, twiddle_real, twiddle_imag );
  input [4:0] ibfp;
  output [4:0] bw_ramwrite;
  input [1:0] ictrl;
  output [1:0] octrl;
  input [8:0] iMemAddr;
  input [31:0] iEvenData;
  input [31:0] iOddData;
  output [8:0] oMemAddr;
  output [31:0] oEvenData;
  output [31:0] oOddData;
  input [16:0] twiddle_real;
  input [16:0] twiddle_imag;
  input clk, rst, clr_bfp, iact;
  output oact;
  wire   iact_calc, oact_calc;
  wire   [31:0] iEvenDataBfp;
  wire   [31:0] iOddDataBfp;
  wire   [1:0] ictrl_calc;
  wire   [8:0] iMemAddrCalc;
  wire   [15:0] opb_imag;
  wire   [15:0] opb_real;
  wire   [15:0] opa_imag;
  wire   [15:0] opa_real;
  wire   [1:0] octrl_calc;
  wire   [8:0] oMemAddrCalc;
  wire   [15:0] dst_opa_real;
  wire   [15:0] dst_opa_imag;
  wire   [15:0] dst_opb_real;
  wire   [15:0] dst_opb_imag;
  tri   clk;
  tri   rst;
  tri   [31:0] iEvenData;
  tri   [31:0] iOddData;
  tri   [16:0] twiddle_real;
  tri   [16:0] twiddle_imag;

  bfp_Shifter_FFT_DW16_FFT_BFPDW5_3 ushifter0 ( .operand(iEvenData[31:16]), 
        .bfp_operand(iEvenDataBfp[31:16]), .bw(ibfp) );
  bfp_Shifter_FFT_DW16_FFT_BFPDW5_2 ushifter1 ( .operand(iEvenData[15:0]), 
        .bfp_operand(iEvenDataBfp[15:0]), .bw(ibfp) );
  bfp_Shifter_FFT_DW16_FFT_BFPDW5_1 ushifter2 ( .operand(iOddData[31:16]), 
        .bfp_operand(iOddDataBfp[31:16]), .bw(ibfp) );
  bfp_Shifter_FFT_DW16_FFT_BFPDW5_0 ushifter3 ( .operand(iOddData[15:0]), 
        .bfp_operand(iOddDataBfp[15:0]), .bw(ibfp) );
  ramPipelineBridge_FFT_N10_FFT_DW16_1 inputStagePipeline ( .clk(clk), .rst(
        rst), .iact(iact), .oact(iact_calc), .ictrl(ictrl), .octrl(ictrl_calc), 
        .iMemAddr(iMemAddr), .iEvenData(iEvenDataBfp), .iOddData(iOddDataBfp), 
        .oMemAddr(iMemAddrCalc), .oEvenData({opa_imag, opa_real}), .oOddData({
        opb_imag, opb_real}) );
  radix2Butterfly_FFT_DW16_FFT_N10_PL_DEPTH3 uradix2bt ( .clk(clk), .rst(rst), 
        .iact(iact_calc), .ictrl(ictrl_calc), .oact(oact_calc), .octrl(
        octrl_calc), .iMemAddr(iMemAddrCalc), .oMemAddr(oMemAddrCalc), 
        .opa_real(opa_real), .opa_imag(opa_imag), .opb_real(opb_real), 
        .opb_imag(opb_imag), .twiddle_real(twiddle_real), .twiddle_imag(
        twiddle_imag), .dst_opa_real(dst_opa_real), .dst_opa_imag(dst_opa_imag), .dst_opb_real(dst_opb_real), .dst_opb_imag(dst_opb_imag) );
  ramPipelineBridge_FFT_N10_FFT_DW16_0 outputStagePipeline ( .clk(clk), .rst(
        rst), .iact(oact_calc), .oact(oact), .ictrl(octrl_calc), .octrl(octrl), 
        .iMemAddr(oMemAddrCalc), .iEvenData({dst_opa_imag, dst_opa_real}), 
        .iOddData({dst_opb_imag, dst_opb_real}), .oMemAddr(oMemAddr), 
        .oEvenData(oEvenData), .oOddData(oOddData) );
  bfp_bitWidthDetector_FFT_BFPDW5_FFT_DW16_0 ubfp_bitWidth ( .operand0(
        oEvenData[31:16]), .operand1(oEvenData[15:0]), .operand2(
        oOddData[31:16]), .operand3(oOddData[15:0]), .bw(bw_ramwrite) );
endmodule


module bfp_maxBitWidth_FFT_BFPDW5_0 ( rst, clk, clr, bw_act, bw, max_bw );
  input [4:0] bw;
  output [4:0] max_bw;
  input rst, clk, clr, bw_act;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n27, n28, n29, n30;
  tri   rst;
  tri   clk;

  DFF_X1 \max_bw_f_reg[4]  ( .D(n30), .CK(clk), .Q(max_bw[4]), .QN(n18) );
  DFF_X1 \max_bw_f_reg[3]  ( .D(n29), .CK(clk), .Q(max_bw[3]), .QN(n20) );
  DFF_X1 \max_bw_f_reg[2]  ( .D(n28), .CK(clk), .Q(max_bw[2]), .QN(n19) );
  DFF_X1 \max_bw_f_reg[1]  ( .D(n27), .CK(clk), .Q(max_bw[1]), .QN(n21) );
  DFF_X1 \max_bw_f_reg[0]  ( .D(n22), .CK(clk), .Q(max_bw[0]) );
  AOI22_X1 U3 ( .A1(bw[4]), .A2(n18), .B1(bw[3]), .B2(n20), .ZN(n6) );
  INV_X1 U4 ( .A(bw[1]), .ZN(n13) );
  OAI222_X1 U5 ( .A1(n13), .A2(max_bw[1]), .B1(n13), .B2(max_bw[0]), .C1(
        max_bw[1]), .C2(max_bw[0]), .ZN(n1) );
  OAI222_X1 U6 ( .A1(n19), .A2(n1), .B1(n19), .B2(bw[2]), .C1(n1), .C2(bw[2]), 
        .ZN(n2) );
  INV_X1 U7 ( .A(n2), .ZN(n3) );
  OAI21_X1 U8 ( .B1(bw[3]), .B2(n20), .A(n3), .ZN(n5) );
  OAI21_X1 U9 ( .B1(bw[4]), .B2(n18), .A(bw_act), .ZN(n4) );
  AOI21_X1 U10 ( .B1(n6), .B2(n5), .A(n4), .ZN(n7) );
  NOR3_X1 U11 ( .A1(n7), .A2(rst), .A3(clr), .ZN(n11) );
  INV_X1 U12 ( .A(n7), .ZN(n8) );
  NOR3_X1 U13 ( .A1(rst), .A2(clr), .A3(n8), .ZN(n12) );
  AOI22_X1 U14 ( .A1(max_bw[3]), .A2(n11), .B1(bw[3]), .B2(n12), .ZN(n9) );
  INV_X1 U15 ( .A(n9), .ZN(n29) );
  AOI22_X1 U16 ( .A1(max_bw[0]), .A2(n11), .B1(n12), .B2(bw[0]), .ZN(n10) );
  INV_X1 U17 ( .A(n10), .ZN(n22) );
  INV_X1 U18 ( .A(n11), .ZN(n17) );
  INV_X1 U19 ( .A(n12), .ZN(n15) );
  OAI22_X1 U20 ( .A1(n21), .A2(n17), .B1(n13), .B2(n15), .ZN(n27) );
  INV_X1 U21 ( .A(bw[2]), .ZN(n14) );
  OAI22_X1 U22 ( .A1(n19), .A2(n17), .B1(n14), .B2(n15), .ZN(n28) );
  INV_X1 U23 ( .A(bw[4]), .ZN(n16) );
  OAI22_X1 U24 ( .A1(n18), .A2(n17), .B1(n16), .B2(n15), .ZN(n30) );
endmodule


module butterflyUnit_FFT_N10_FFT_DW16_FFT_BFPDW5_PL_DEPTH3 ( clk, rst, clr_bfp, 
        ibfp, obfp, iact, oact, ictrl, octrl, MemAddr, twiddleFactorAddr, 
        evenOdd, ifft, twact, twa, twdr_cos, ract_ram0, ra_ram0, rdr_ram0, 
        wact_ram0, wa_ram0, wdw_ram0, ract_ram1, ra_ram1, rdr_ram1, wact_ram1, 
        wa_ram1, wdw_ram1 );
  input [4:0] ibfp;
  output [4:0] obfp;
  input [1:0] ictrl;
  output [1:0] octrl;
  input [8:0] MemAddr;
  input [8:0] twiddleFactorAddr;
  output [7:0] twa;
  input [15:0] twdr_cos;
  output [8:0] ra_ram0;
  input [31:0] rdr_ram0;
  output [8:0] wa_ram0;
  output [31:0] wdw_ram0;
  output [8:0] ra_ram1;
  input [31:0] rdr_ram1;
  output [8:0] wa_ram1;
  output [31:0] wdw_ram1;
  input clk, rst, clr_bfp, iact, evenOdd, ifft;
  output oact, twact, ract_ram0, wact_ram0, ract_ram1, wact_ram1;
  wire   wact_ram0, act, N6, N7, N8, oactCore, N11, n1;
  wire   [1:0] ctrl;
  wire   [8:0] iMemAddr;
  wire   [4:0] bw_ramwrite;
  wire   [1:0] octrlCore;
  wire   [8:0] oMemAddr;
  wire   [31:0] oEvenData;
  wire   [31:0] oOddData;
  wire   [4:0] bw_ramwrite_dly;
  tri   clk;
  tri   rst;
  tri   iact;
  tri   [8:0] twiddleFactorAddr;
  tri   evenOdd;
  tri   ifft;
  tri   twact;
  tri   [7:0] twa;
  tri   [15:0] twdr_cos;
  tri   [31:0] rdr_ram0;
  tri   [31:0] rdr_ram1;
  tri   [16:0] tdr_rom_real;
  tri   [16:0] tdr_rom_imag;
  assign ract_ram1 = iact;
  assign ract_ram0 = iact;
  assign ra_ram1[8] = MemAddr[8];
  assign ra_ram0[8] = MemAddr[8];
  assign ra_ram1[7] = MemAddr[7];
  assign ra_ram0[7] = MemAddr[7];
  assign ra_ram1[6] = MemAddr[6];
  assign ra_ram0[6] = MemAddr[6];
  assign ra_ram1[5] = MemAddr[5];
  assign ra_ram0[5] = MemAddr[5];
  assign ra_ram1[4] = MemAddr[4];
  assign ra_ram0[4] = MemAddr[4];
  assign ra_ram1[3] = MemAddr[3];
  assign ra_ram0[3] = MemAddr[3];
  assign ra_ram1[2] = MemAddr[2];
  assign ra_ram0[2] = MemAddr[2];
  assign ra_ram1[1] = MemAddr[1];
  assign ra_ram0[1] = MemAddr[1];
  assign ra_ram1[0] = MemAddr[0];
  assign ra_ram0[0] = MemAddr[0];
  assign wact_ram1 = wact_ram0;
  assign wa_ram1[8] = wa_ram0[8];
  assign wa_ram1[7] = wa_ram0[7];
  assign wa_ram1[6] = wa_ram0[6];
  assign wa_ram1[5] = wa_ram0[5];
  assign wa_ram1[4] = wa_ram0[4];
  assign wa_ram1[3] = wa_ram0[3];
  assign wa_ram1[2] = wa_ram0[2];
  assign wa_ram1[1] = wa_ram0[1];
  assign wa_ram1[0] = wa_ram0[0];

  twiddleFactorRomBridge utwiddleFactorRomBridge ( .clk(clk), .rst(rst), 
        .tact_rom(iact), .evenOdd(evenOdd), .ifft(ifft), .ta_rom(
        twiddleFactorAddr), .tdr_rom_real(tdr_rom_real), .tdr_rom_imag(
        tdr_rom_imag), .twact(twact), .twa(twa), .twdr_cos(twdr_cos) );
  butterflyCore_FFT_N10_FFT_DW16_FFT_BFPDW5_PL_DEPTH3 ubutterflyCore ( .clk(
        clk), .rst(rst), .clr_bfp(clr_bfp), .ibfp(ibfp), .bw_ramwrite(
        bw_ramwrite), .iact(act), .oact(oactCore), .ictrl(ctrl), .octrl(
        octrlCore), .iMemAddr(iMemAddr), .iEvenData(rdr_ram0), .iOddData(
        rdr_ram1), .oMemAddr(oMemAddr), .oEvenData(oEvenData), .oOddData(
        oOddData), .twiddle_real(tdr_rom_real), .twiddle_imag(tdr_rom_imag) );
  bfp_maxBitWidth_FFT_BFPDW5_0 ubfp_maxBitWidth ( .rst(rst), .clk(clk), .clr(
        clr_bfp), .bw_act(oact), .bw(bw_ramwrite_dly), .max_bw(obfp) );
  DFF_X1 \ctrl_reg[1]  ( .D(N8), .CK(clk), .Q(ctrl[1]) );
  DFF_X1 \ctrl_reg[0]  ( .D(N7), .CK(clk), .Q(ctrl[0]) );
  DFF_X1 act_reg ( .D(N6), .CK(clk), .Q(act) );
  DFF_X1 \iMemAddr_reg[8]  ( .D(MemAddr[8]), .CK(clk), .Q(iMemAddr[8]) );
  DFF_X1 \iMemAddr_reg[7]  ( .D(MemAddr[7]), .CK(clk), .Q(iMemAddr[7]) );
  DFF_X1 \iMemAddr_reg[6]  ( .D(MemAddr[6]), .CK(clk), .Q(iMemAddr[6]) );
  DFF_X1 \iMemAddr_reg[5]  ( .D(MemAddr[5]), .CK(clk), .Q(iMemAddr[5]) );
  DFF_X1 \iMemAddr_reg[4]  ( .D(MemAddr[4]), .CK(clk), .Q(iMemAddr[4]) );
  DFF_X1 \iMemAddr_reg[3]  ( .D(MemAddr[3]), .CK(clk), .Q(iMemAddr[3]) );
  DFF_X1 \iMemAddr_reg[2]  ( .D(MemAddr[2]), .CK(clk), .Q(iMemAddr[2]) );
  DFF_X1 \iMemAddr_reg[1]  ( .D(MemAddr[1]), .CK(clk), .Q(iMemAddr[1]) );
  DFF_X1 \iMemAddr_reg[0]  ( .D(MemAddr[0]), .CK(clk), .Q(iMemAddr[0]) );
  DFF_X1 oact_reg ( .D(N11), .CK(clk), .Q(oact) );
  DFF_X1 \octrl_reg[1]  ( .D(octrlCore[1]), .CK(clk), .Q(octrl[1]) );
  DFF_X1 \octrl_reg[0]  ( .D(octrlCore[0]), .CK(clk), .Q(octrl[0]) );
  DFF_X1 \wdw_ram0_reg[31]  ( .D(oEvenData[31]), .CK(clk), .Q(wdw_ram0[31]) );
  DFF_X1 \wdw_ram0_reg[30]  ( .D(oEvenData[30]), .CK(clk), .Q(wdw_ram0[30]) );
  DFF_X1 \wdw_ram0_reg[29]  ( .D(oEvenData[29]), .CK(clk), .Q(wdw_ram0[29]) );
  DFF_X1 \wdw_ram0_reg[28]  ( .D(oEvenData[28]), .CK(clk), .Q(wdw_ram0[28]) );
  DFF_X1 \wdw_ram0_reg[27]  ( .D(oEvenData[27]), .CK(clk), .Q(wdw_ram0[27]) );
  DFF_X1 \wdw_ram0_reg[26]  ( .D(oEvenData[26]), .CK(clk), .Q(wdw_ram0[26]) );
  DFF_X1 \wdw_ram0_reg[25]  ( .D(oEvenData[25]), .CK(clk), .Q(wdw_ram0[25]) );
  DFF_X1 \wdw_ram0_reg[24]  ( .D(oEvenData[24]), .CK(clk), .Q(wdw_ram0[24]) );
  DFF_X1 \wdw_ram0_reg[23]  ( .D(oEvenData[23]), .CK(clk), .Q(wdw_ram0[23]) );
  DFF_X1 \wdw_ram0_reg[22]  ( .D(oEvenData[22]), .CK(clk), .Q(wdw_ram0[22]) );
  DFF_X1 \wdw_ram0_reg[21]  ( .D(oEvenData[21]), .CK(clk), .Q(wdw_ram0[21]) );
  DFF_X1 \wdw_ram0_reg[20]  ( .D(oEvenData[20]), .CK(clk), .Q(wdw_ram0[20]) );
  DFF_X1 \wdw_ram0_reg[19]  ( .D(oEvenData[19]), .CK(clk), .Q(wdw_ram0[19]) );
  DFF_X1 \wdw_ram0_reg[18]  ( .D(oEvenData[18]), .CK(clk), .Q(wdw_ram0[18]) );
  DFF_X1 \wdw_ram0_reg[17]  ( .D(oEvenData[17]), .CK(clk), .Q(wdw_ram0[17]) );
  DFF_X1 \wdw_ram0_reg[16]  ( .D(oEvenData[16]), .CK(clk), .Q(wdw_ram0[16]) );
  DFF_X1 \wdw_ram0_reg[15]  ( .D(oEvenData[15]), .CK(clk), .Q(wdw_ram0[15]) );
  DFF_X1 \wdw_ram0_reg[14]  ( .D(oEvenData[14]), .CK(clk), .Q(wdw_ram0[14]) );
  DFF_X1 \wdw_ram0_reg[13]  ( .D(oEvenData[13]), .CK(clk), .Q(wdw_ram0[13]) );
  DFF_X1 \wdw_ram0_reg[12]  ( .D(oEvenData[12]), .CK(clk), .Q(wdw_ram0[12]) );
  DFF_X1 \wdw_ram0_reg[11]  ( .D(oEvenData[11]), .CK(clk), .Q(wdw_ram0[11]) );
  DFF_X1 \wdw_ram0_reg[10]  ( .D(oEvenData[10]), .CK(clk), .Q(wdw_ram0[10]) );
  DFF_X1 \wdw_ram0_reg[9]  ( .D(oEvenData[9]), .CK(clk), .Q(wdw_ram0[9]) );
  DFF_X1 \wdw_ram0_reg[8]  ( .D(oEvenData[8]), .CK(clk), .Q(wdw_ram0[8]) );
  DFF_X1 \wdw_ram0_reg[7]  ( .D(oEvenData[7]), .CK(clk), .Q(wdw_ram0[7]) );
  DFF_X1 \wdw_ram0_reg[6]  ( .D(oEvenData[6]), .CK(clk), .Q(wdw_ram0[6]) );
  DFF_X1 \wdw_ram0_reg[5]  ( .D(oEvenData[5]), .CK(clk), .Q(wdw_ram0[5]) );
  DFF_X1 \wdw_ram0_reg[4]  ( .D(oEvenData[4]), .CK(clk), .Q(wdw_ram0[4]) );
  DFF_X1 \wdw_ram0_reg[3]  ( .D(oEvenData[3]), .CK(clk), .Q(wdw_ram0[3]) );
  DFF_X1 \wdw_ram0_reg[2]  ( .D(oEvenData[2]), .CK(clk), .Q(wdw_ram0[2]) );
  DFF_X1 \wdw_ram0_reg[1]  ( .D(oEvenData[1]), .CK(clk), .Q(wdw_ram0[1]) );
  DFF_X1 \wdw_ram0_reg[0]  ( .D(oEvenData[0]), .CK(clk), .Q(wdw_ram0[0]) );
  DFF_X1 wact_ram1_reg ( .D(oactCore), .CK(clk), .Q(wact_ram0) );
  DFF_X1 \wa_ram1_reg[8]  ( .D(oMemAddr[8]), .CK(clk), .Q(wa_ram0[8]) );
  DFF_X1 \wa_ram1_reg[7]  ( .D(oMemAddr[7]), .CK(clk), .Q(wa_ram0[7]) );
  DFF_X1 \wa_ram1_reg[6]  ( .D(oMemAddr[6]), .CK(clk), .Q(wa_ram0[6]) );
  DFF_X1 \wa_ram1_reg[5]  ( .D(oMemAddr[5]), .CK(clk), .Q(wa_ram0[5]) );
  DFF_X1 \wa_ram1_reg[4]  ( .D(oMemAddr[4]), .CK(clk), .Q(wa_ram0[4]) );
  DFF_X1 \wa_ram1_reg[3]  ( .D(oMemAddr[3]), .CK(clk), .Q(wa_ram0[3]) );
  DFF_X1 \wa_ram1_reg[2]  ( .D(oMemAddr[2]), .CK(clk), .Q(wa_ram0[2]) );
  DFF_X1 \wa_ram1_reg[1]  ( .D(oMemAddr[1]), .CK(clk), .Q(wa_ram0[1]) );
  DFF_X1 \wa_ram1_reg[0]  ( .D(oMemAddr[0]), .CK(clk), .Q(wa_ram0[0]) );
  DFF_X1 \wdw_ram1_reg[31]  ( .D(oOddData[31]), .CK(clk), .Q(wdw_ram1[31]) );
  DFF_X1 \wdw_ram1_reg[30]  ( .D(oOddData[30]), .CK(clk), .Q(wdw_ram1[30]) );
  DFF_X1 \wdw_ram1_reg[29]  ( .D(oOddData[29]), .CK(clk), .Q(wdw_ram1[29]) );
  DFF_X1 \wdw_ram1_reg[28]  ( .D(oOddData[28]), .CK(clk), .Q(wdw_ram1[28]) );
  DFF_X1 \wdw_ram1_reg[27]  ( .D(oOddData[27]), .CK(clk), .Q(wdw_ram1[27]) );
  DFF_X1 \wdw_ram1_reg[26]  ( .D(oOddData[26]), .CK(clk), .Q(wdw_ram1[26]) );
  DFF_X1 \wdw_ram1_reg[25]  ( .D(oOddData[25]), .CK(clk), .Q(wdw_ram1[25]) );
  DFF_X1 \wdw_ram1_reg[24]  ( .D(oOddData[24]), .CK(clk), .Q(wdw_ram1[24]) );
  DFF_X1 \wdw_ram1_reg[23]  ( .D(oOddData[23]), .CK(clk), .Q(wdw_ram1[23]) );
  DFF_X1 \wdw_ram1_reg[22]  ( .D(oOddData[22]), .CK(clk), .Q(wdw_ram1[22]) );
  DFF_X1 \wdw_ram1_reg[21]  ( .D(oOddData[21]), .CK(clk), .Q(wdw_ram1[21]) );
  DFF_X1 \wdw_ram1_reg[20]  ( .D(oOddData[20]), .CK(clk), .Q(wdw_ram1[20]) );
  DFF_X1 \wdw_ram1_reg[19]  ( .D(oOddData[19]), .CK(clk), .Q(wdw_ram1[19]) );
  DFF_X1 \wdw_ram1_reg[18]  ( .D(oOddData[18]), .CK(clk), .Q(wdw_ram1[18]) );
  DFF_X1 \wdw_ram1_reg[17]  ( .D(oOddData[17]), .CK(clk), .Q(wdw_ram1[17]) );
  DFF_X1 \wdw_ram1_reg[16]  ( .D(oOddData[16]), .CK(clk), .Q(wdw_ram1[16]) );
  DFF_X1 \wdw_ram1_reg[15]  ( .D(oOddData[15]), .CK(clk), .Q(wdw_ram1[15]) );
  DFF_X1 \wdw_ram1_reg[14]  ( .D(oOddData[14]), .CK(clk), .Q(wdw_ram1[14]) );
  DFF_X1 \wdw_ram1_reg[13]  ( .D(oOddData[13]), .CK(clk), .Q(wdw_ram1[13]) );
  DFF_X1 \wdw_ram1_reg[12]  ( .D(oOddData[12]), .CK(clk), .Q(wdw_ram1[12]) );
  DFF_X1 \wdw_ram1_reg[11]  ( .D(oOddData[11]), .CK(clk), .Q(wdw_ram1[11]) );
  DFF_X1 \wdw_ram1_reg[10]  ( .D(oOddData[10]), .CK(clk), .Q(wdw_ram1[10]) );
  DFF_X1 \wdw_ram1_reg[9]  ( .D(oOddData[9]), .CK(clk), .Q(wdw_ram1[9]) );
  DFF_X1 \wdw_ram1_reg[8]  ( .D(oOddData[8]), .CK(clk), .Q(wdw_ram1[8]) );
  DFF_X1 \wdw_ram1_reg[7]  ( .D(oOddData[7]), .CK(clk), .Q(wdw_ram1[7]) );
  DFF_X1 \wdw_ram1_reg[6]  ( .D(oOddData[6]), .CK(clk), .Q(wdw_ram1[6]) );
  DFF_X1 \wdw_ram1_reg[5]  ( .D(oOddData[5]), .CK(clk), .Q(wdw_ram1[5]) );
  DFF_X1 \wdw_ram1_reg[4]  ( .D(oOddData[4]), .CK(clk), .Q(wdw_ram1[4]) );
  DFF_X1 \wdw_ram1_reg[3]  ( .D(oOddData[3]), .CK(clk), .Q(wdw_ram1[3]) );
  DFF_X1 \wdw_ram1_reg[2]  ( .D(oOddData[2]), .CK(clk), .Q(wdw_ram1[2]) );
  DFF_X1 \wdw_ram1_reg[1]  ( .D(oOddData[1]), .CK(clk), .Q(wdw_ram1[1]) );
  DFF_X1 \wdw_ram1_reg[0]  ( .D(oOddData[0]), .CK(clk), .Q(wdw_ram1[0]) );
  DFF_X1 \bw_ramwrite_dly_reg[4]  ( .D(bw_ramwrite[4]), .CK(clk), .Q(
        bw_ramwrite_dly[4]) );
  DFF_X1 \bw_ramwrite_dly_reg[3]  ( .D(bw_ramwrite[3]), .CK(clk), .Q(
        bw_ramwrite_dly[3]) );
  DFF_X1 \bw_ramwrite_dly_reg[2]  ( .D(bw_ramwrite[2]), .CK(clk), .Q(
        bw_ramwrite_dly[2]) );
  DFF_X1 \bw_ramwrite_dly_reg[1]  ( .D(bw_ramwrite[1]), .CK(clk), .Q(
        bw_ramwrite_dly[1]) );
  DFF_X1 \bw_ramwrite_dly_reg[0]  ( .D(bw_ramwrite[0]), .CK(clk), .Q(
        bw_ramwrite_dly[0]) );
  INV_X1 U3 ( .A(rst), .ZN(n1) );
  AND2_X1 U5 ( .A1(n1), .A2(iact), .ZN(N6) );
  AND2_X1 U4 ( .A1(n1), .A2(oactCore), .ZN(N11) );
  AND2_X1 U6 ( .A1(n1), .A2(ictrl[0]), .ZN(N7) );
  AND2_X1 U7 ( .A1(n1), .A2(ictrl[1]), .ZN(N8) );
endmodule



    module readBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_1 ( 
        mode, ract_fft, ra_fft, rdr_fft, ract_dma, ra_dma, rdr_dma, ract_ram, 
        ra_ram, rdr_ram );
  input [1:0] mode;
  input [8:0] ra_fft;
  output [31:0] rdr_fft;
  input [8:0] ra_dma;
  output [31:0] rdr_dma;
  output [8:0] ra_ram;
  input [31:0] rdr_ram;
  input ract_fft, ract_dma;
  output ract_ram;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13;
  tri   ract_fft;
  tri   [31:0] rdr_ram;
  tri   \ra_ram[0] ;
  tri   \ra_ram[1] ;
  tri   \ra_ram[2] ;
  tri   \ra_ram[3] ;
  tri   \ra_ram[4] ;
  tri   \ra_ram[5] ;
  tri   \ra_ram[6] ;
  tri   \ra_ram[7] ;
  tri   \ra_ram[8] ;
  tri   ract_ram;
  assign rdr_dma[31] = rdr_ram[31];
  assign rdr_fft[31] = rdr_ram[31];
  assign rdr_dma[30] = rdr_ram[30];
  assign rdr_fft[30] = rdr_ram[30];
  assign rdr_dma[29] = rdr_ram[29];
  assign rdr_fft[29] = rdr_ram[29];
  assign rdr_dma[28] = rdr_ram[28];
  assign rdr_fft[28] = rdr_ram[28];
  assign rdr_dma[27] = rdr_ram[27];
  assign rdr_fft[27] = rdr_ram[27];
  assign rdr_dma[26] = rdr_ram[26];
  assign rdr_fft[26] = rdr_ram[26];
  assign rdr_dma[25] = rdr_ram[25];
  assign rdr_fft[25] = rdr_ram[25];
  assign rdr_dma[24] = rdr_ram[24];
  assign rdr_fft[24] = rdr_ram[24];
  assign rdr_dma[23] = rdr_ram[23];
  assign rdr_fft[23] = rdr_ram[23];
  assign rdr_dma[22] = rdr_ram[22];
  assign rdr_fft[22] = rdr_ram[22];
  assign rdr_dma[21] = rdr_ram[21];
  assign rdr_fft[21] = rdr_ram[21];
  assign rdr_dma[20] = rdr_ram[20];
  assign rdr_fft[20] = rdr_ram[20];
  assign rdr_dma[19] = rdr_ram[19];
  assign rdr_fft[19] = rdr_ram[19];
  assign rdr_dma[18] = rdr_ram[18];
  assign rdr_fft[18] = rdr_ram[18];
  assign rdr_dma[17] = rdr_ram[17];
  assign rdr_fft[17] = rdr_ram[17];
  assign rdr_dma[16] = rdr_ram[16];
  assign rdr_fft[16] = rdr_ram[16];
  assign rdr_dma[15] = rdr_ram[15];
  assign rdr_fft[15] = rdr_ram[15];
  assign rdr_dma[14] = rdr_ram[14];
  assign rdr_fft[14] = rdr_ram[14];
  assign rdr_dma[13] = rdr_ram[13];
  assign rdr_fft[13] = rdr_ram[13];
  assign rdr_dma[12] = rdr_ram[12];
  assign rdr_fft[12] = rdr_ram[12];
  assign rdr_dma[11] = rdr_ram[11];
  assign rdr_fft[11] = rdr_ram[11];
  assign rdr_dma[10] = rdr_ram[10];
  assign rdr_fft[10] = rdr_ram[10];
  assign rdr_dma[9] = rdr_ram[9];
  assign rdr_fft[9] = rdr_ram[9];
  assign rdr_dma[8] = rdr_ram[8];
  assign rdr_fft[8] = rdr_ram[8];
  assign rdr_dma[7] = rdr_ram[7];
  assign rdr_fft[7] = rdr_ram[7];
  assign rdr_dma[6] = rdr_ram[6];
  assign rdr_fft[6] = rdr_ram[6];
  assign rdr_dma[5] = rdr_ram[5];
  assign rdr_fft[5] = rdr_ram[5];
  assign rdr_dma[4] = rdr_ram[4];
  assign rdr_fft[4] = rdr_ram[4];
  assign rdr_dma[3] = rdr_ram[3];
  assign rdr_fft[3] = rdr_ram[3];
  assign rdr_dma[2] = rdr_ram[2];
  assign rdr_fft[2] = rdr_ram[2];
  assign rdr_dma[1] = rdr_ram[1];
  assign rdr_fft[1] = rdr_ram[1];
  assign rdr_dma[0] = rdr_ram[0];
  assign rdr_fft[0] = rdr_ram[0];

  INV_X1 U2 ( .A(mode[1]), .ZN(n1) );
  AND2_X1 U3 ( .A1(n1), .A2(mode[0]), .ZN(n12) );
  NOR2_X1 U4 ( .A1(mode[0]), .A2(n1), .ZN(n11) );
  AOI22_X1 U5 ( .A1(n12), .A2(ract_fft), .B1(n11), .B2(ract_dma), .ZN(n2) );
  INV_X1 U6 ( .A(n2), .ZN(ract_ram) );
  AOI22_X1 U7 ( .A1(n12), .A2(ra_fft[8]), .B1(n11), .B2(ra_dma[8]), .ZN(n3) );
  INV_X1 U8 ( .A(n3), .ZN(ra_ram[8]) );
  AOI22_X1 U9 ( .A1(n12), .A2(ra_fft[7]), .B1(n11), .B2(ra_dma[7]), .ZN(n4) );
  INV_X1 U10 ( .A(n4), .ZN(ra_ram[7]) );
  AOI22_X1 U11 ( .A1(n12), .A2(ra_fft[6]), .B1(n11), .B2(ra_dma[6]), .ZN(n5)
         );
  INV_X1 U12 ( .A(n5), .ZN(ra_ram[6]) );
  AOI22_X1 U13 ( .A1(n12), .A2(ra_fft[5]), .B1(n11), .B2(ra_dma[5]), .ZN(n6)
         );
  INV_X1 U14 ( .A(n6), .ZN(ra_ram[5]) );
  AOI22_X1 U15 ( .A1(n12), .A2(ra_fft[4]), .B1(n11), .B2(ra_dma[4]), .ZN(n7)
         );
  INV_X1 U16 ( .A(n7), .ZN(ra_ram[4]) );
  AOI22_X1 U17 ( .A1(n12), .A2(ra_fft[3]), .B1(n11), .B2(ra_dma[3]), .ZN(n8)
         );
  INV_X1 U18 ( .A(n8), .ZN(ra_ram[3]) );
  AOI22_X1 U19 ( .A1(n12), .A2(ra_fft[2]), .B1(n11), .B2(ra_dma[2]), .ZN(n9)
         );
  INV_X1 U20 ( .A(n9), .ZN(ra_ram[2]) );
  AOI22_X1 U21 ( .A1(n12), .A2(ra_fft[1]), .B1(n11), .B2(ra_dma[1]), .ZN(n10)
         );
  INV_X1 U22 ( .A(n10), .ZN(ra_ram[1]) );
  AOI22_X1 U23 ( .A1(n12), .A2(ra_fft[0]), .B1(n11), .B2(ra_dma[0]), .ZN(n13)
         );
  INV_X1 U24 ( .A(n13), .ZN(ra_ram[0]) );
endmodule



    module writeBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_1 ( 
        mode, wact_fft, wa_fft, wdw_fft, wact_istream, wa_istream, wdw_istream, 
        wact_ram, wa_ram, wdw_ram );
  input [1:0] mode;
  input [8:0] wa_fft;
  input [31:0] wdw_fft;
  input [8:0] wa_istream;
  input [31:0] wdw_istream;
  output [8:0] wa_ram;
  output [31:0] wdw_ram;
  input wact_fft, wact_istream;
  output wact_ram;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47;
  tri   wact_ram;
  tri   [8:0] wa_ram;
  tri   [31:0] wdw_ram;

  INV_X1 U2 ( .A(mode[0]), .ZN(n1) );
  NOR2_X2 U3 ( .A1(mode[1]), .A2(n1), .ZN(n38) );
  CLKBUF_X1 U4 ( .A(n38), .Z(n46) );
  NOR2_X2 U5 ( .A1(mode[1]), .A2(mode[0]), .ZN(n37) );
  CLKBUF_X1 U6 ( .A(n37), .Z(n45) );
  AOI22_X1 U7 ( .A1(n46), .A2(wact_fft), .B1(n45), .B2(wact_istream), .ZN(n2)
         );
  INV_X1 U8 ( .A(n2), .ZN(wact_ram) );
  AOI22_X1 U9 ( .A1(n38), .A2(wdw_fft[0]), .B1(n37), .B2(wdw_istream[0]), .ZN(
        n3) );
  INV_X1 U10 ( .A(n3), .ZN(wdw_ram[0]) );
  AOI22_X1 U11 ( .A1(n46), .A2(wdw_fft[1]), .B1(n45), .B2(wdw_istream[1]), 
        .ZN(n4) );
  INV_X1 U12 ( .A(n4), .ZN(wdw_ram[1]) );
  AOI22_X1 U13 ( .A1(n38), .A2(wdw_fft[2]), .B1(n37), .B2(wdw_istream[2]), 
        .ZN(n5) );
  INV_X1 U14 ( .A(n5), .ZN(wdw_ram[2]) );
  AOI22_X1 U15 ( .A1(n46), .A2(wdw_fft[3]), .B1(n45), .B2(wdw_istream[3]), 
        .ZN(n6) );
  INV_X1 U16 ( .A(n6), .ZN(wdw_ram[3]) );
  AOI22_X1 U17 ( .A1(n38), .A2(wdw_fft[4]), .B1(n37), .B2(wdw_istream[4]), 
        .ZN(n7) );
  INV_X1 U18 ( .A(n7), .ZN(wdw_ram[4]) );
  AOI22_X1 U19 ( .A1(n46), .A2(wdw_fft[5]), .B1(n45), .B2(wdw_istream[5]), 
        .ZN(n8) );
  INV_X1 U20 ( .A(n8), .ZN(wdw_ram[5]) );
  AOI22_X1 U21 ( .A1(n38), .A2(wdw_fft[6]), .B1(n37), .B2(wdw_istream[6]), 
        .ZN(n9) );
  INV_X1 U22 ( .A(n9), .ZN(wdw_ram[6]) );
  AOI22_X1 U23 ( .A1(n38), .A2(wdw_fft[7]), .B1(n37), .B2(wdw_istream[7]), 
        .ZN(n10) );
  INV_X1 U24 ( .A(n10), .ZN(wdw_ram[7]) );
  AOI22_X1 U25 ( .A1(n38), .A2(wdw_fft[8]), .B1(n37), .B2(wdw_istream[8]), 
        .ZN(n11) );
  INV_X1 U26 ( .A(n11), .ZN(wdw_ram[8]) );
  AOI22_X1 U27 ( .A1(n38), .A2(wdw_fft[9]), .B1(n37), .B2(wdw_istream[9]), 
        .ZN(n12) );
  INV_X1 U28 ( .A(n12), .ZN(wdw_ram[9]) );
  AOI22_X1 U29 ( .A1(n38), .A2(wdw_fft[10]), .B1(n37), .B2(wdw_istream[10]), 
        .ZN(n13) );
  INV_X1 U30 ( .A(n13), .ZN(wdw_ram[10]) );
  AOI22_X1 U31 ( .A1(n38), .A2(wdw_fft[11]), .B1(n37), .B2(wdw_istream[11]), 
        .ZN(n14) );
  INV_X1 U32 ( .A(n14), .ZN(wdw_ram[11]) );
  AOI22_X1 U33 ( .A1(n38), .A2(wdw_fft[12]), .B1(n37), .B2(wdw_istream[12]), 
        .ZN(n15) );
  INV_X1 U34 ( .A(n15), .ZN(wdw_ram[12]) );
  AOI22_X1 U35 ( .A1(n38), .A2(wdw_fft[13]), .B1(n37), .B2(wdw_istream[13]), 
        .ZN(n16) );
  INV_X1 U36 ( .A(n16), .ZN(wdw_ram[13]) );
  AOI22_X1 U37 ( .A1(n38), .A2(wdw_fft[14]), .B1(n37), .B2(wdw_istream[14]), 
        .ZN(n17) );
  INV_X1 U38 ( .A(n17), .ZN(wdw_ram[14]) );
  AOI22_X1 U39 ( .A1(n38), .A2(wdw_fft[15]), .B1(n37), .B2(wdw_istream[15]), 
        .ZN(n18) );
  INV_X1 U40 ( .A(n18), .ZN(wdw_ram[15]) );
  AOI22_X1 U41 ( .A1(n38), .A2(wdw_fft[16]), .B1(n37), .B2(wdw_istream[16]), 
        .ZN(n19) );
  INV_X1 U42 ( .A(n19), .ZN(wdw_ram[16]) );
  AOI22_X1 U43 ( .A1(n46), .A2(wdw_fft[17]), .B1(n45), .B2(wdw_istream[17]), 
        .ZN(n20) );
  INV_X1 U44 ( .A(n20), .ZN(wdw_ram[17]) );
  AOI22_X1 U45 ( .A1(n38), .A2(wdw_fft[18]), .B1(n37), .B2(wdw_istream[18]), 
        .ZN(n21) );
  INV_X1 U46 ( .A(n21), .ZN(wdw_ram[18]) );
  AOI22_X1 U47 ( .A1(n46), .A2(wdw_fft[19]), .B1(n45), .B2(wdw_istream[19]), 
        .ZN(n22) );
  INV_X1 U48 ( .A(n22), .ZN(wdw_ram[19]) );
  AOI22_X1 U49 ( .A1(n38), .A2(wdw_fft[20]), .B1(n37), .B2(wdw_istream[20]), 
        .ZN(n23) );
  INV_X1 U50 ( .A(n23), .ZN(wdw_ram[20]) );
  AOI22_X1 U51 ( .A1(n46), .A2(wdw_fft[21]), .B1(n45), .B2(wdw_istream[21]), 
        .ZN(n24) );
  INV_X1 U52 ( .A(n24), .ZN(wdw_ram[21]) );
  AOI22_X1 U53 ( .A1(n38), .A2(wdw_fft[22]), .B1(n37), .B2(wdw_istream[22]), 
        .ZN(n25) );
  INV_X1 U54 ( .A(n25), .ZN(wdw_ram[22]) );
  AOI22_X1 U55 ( .A1(n46), .A2(wdw_fft[23]), .B1(n45), .B2(wdw_istream[23]), 
        .ZN(n26) );
  INV_X1 U56 ( .A(n26), .ZN(wdw_ram[23]) );
  AOI22_X1 U57 ( .A1(n46), .A2(wdw_fft[24]), .B1(n45), .B2(wdw_istream[24]), 
        .ZN(n27) );
  INV_X1 U58 ( .A(n27), .ZN(wdw_ram[24]) );
  AOI22_X1 U59 ( .A1(n38), .A2(wdw_fft[25]), .B1(n45), .B2(wdw_istream[25]), 
        .ZN(n28) );
  INV_X1 U60 ( .A(n28), .ZN(wdw_ram[25]) );
  AOI22_X1 U61 ( .A1(n46), .A2(wdw_fft[26]), .B1(n37), .B2(wdw_istream[26]), 
        .ZN(n29) );
  INV_X1 U62 ( .A(n29), .ZN(wdw_ram[26]) );
  AOI22_X1 U63 ( .A1(n46), .A2(wdw_fft[27]), .B1(n45), .B2(wdw_istream[27]), 
        .ZN(n30) );
  INV_X1 U64 ( .A(n30), .ZN(wdw_ram[27]) );
  AOI22_X1 U65 ( .A1(n46), .A2(wdw_fft[28]), .B1(n45), .B2(wdw_istream[28]), 
        .ZN(n31) );
  INV_X1 U66 ( .A(n31), .ZN(wdw_ram[28]) );
  AOI22_X1 U67 ( .A1(n46), .A2(wdw_fft[29]), .B1(n45), .B2(wdw_istream[29]), 
        .ZN(n32) );
  INV_X1 U68 ( .A(n32), .ZN(wdw_ram[29]) );
  AOI22_X1 U69 ( .A1(n46), .A2(wdw_fft[30]), .B1(n45), .B2(wdw_istream[30]), 
        .ZN(n33) );
  INV_X1 U70 ( .A(n33), .ZN(wdw_ram[30]) );
  AOI22_X1 U71 ( .A1(n46), .A2(wdw_fft[31]), .B1(n45), .B2(wdw_istream[31]), 
        .ZN(n34) );
  INV_X1 U72 ( .A(n34), .ZN(wdw_ram[31]) );
  AOI22_X1 U73 ( .A1(n38), .A2(wa_fft[0]), .B1(n37), .B2(wa_istream[0]), .ZN(
        n35) );
  INV_X1 U74 ( .A(n35), .ZN(wa_ram[0]) );
  AOI22_X1 U75 ( .A1(n46), .A2(wa_fft[1]), .B1(n45), .B2(wa_istream[1]), .ZN(
        n36) );
  INV_X1 U76 ( .A(n36), .ZN(wa_ram[1]) );
  AOI22_X1 U77 ( .A1(n38), .A2(wa_fft[2]), .B1(n37), .B2(wa_istream[2]), .ZN(
        n39) );
  INV_X1 U78 ( .A(n39), .ZN(wa_ram[2]) );
  AOI22_X1 U79 ( .A1(n46), .A2(wa_fft[3]), .B1(n45), .B2(wa_istream[3]), .ZN(
        n40) );
  INV_X1 U80 ( .A(n40), .ZN(wa_ram[3]) );
  AOI22_X1 U81 ( .A1(n46), .A2(wa_fft[4]), .B1(n45), .B2(wa_istream[4]), .ZN(
        n41) );
  INV_X1 U82 ( .A(n41), .ZN(wa_ram[4]) );
  AOI22_X1 U83 ( .A1(n46), .A2(wa_fft[5]), .B1(n45), .B2(wa_istream[5]), .ZN(
        n42) );
  INV_X1 U84 ( .A(n42), .ZN(wa_ram[5]) );
  AOI22_X1 U85 ( .A1(n46), .A2(wa_fft[6]), .B1(n45), .B2(wa_istream[6]), .ZN(
        n43) );
  INV_X1 U86 ( .A(n43), .ZN(wa_ram[6]) );
  AOI22_X1 U87 ( .A1(n46), .A2(wa_fft[7]), .B1(n45), .B2(wa_istream[7]), .ZN(
        n44) );
  INV_X1 U88 ( .A(n44), .ZN(wa_ram[7]) );
  AOI22_X1 U89 ( .A1(n46), .A2(wa_fft[8]), .B1(n45), .B2(wa_istream[8]), .ZN(
        n47) );
  INV_X1 U90 ( .A(n47), .ZN(wa_ram[8]) );
endmodule



    module writeBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_0 ( 
        mode, wact_fft, wa_fft, wdw_fft, wact_istream, wa_istream, wdw_istream, 
        wact_ram, wa_ram, wdw_ram );
  input [1:0] mode;
  input [8:0] wa_fft;
  input [31:0] wdw_fft;
  input [8:0] wa_istream;
  input [31:0] wdw_istream;
  output [8:0] wa_ram;
  output [31:0] wdw_ram;
  input wact_fft, wact_istream;
  output wact_ram;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47;
  tri   wact_ram;
  tri   [8:0] wa_ram;
  tri   [31:0] wdw_ram;

  INV_X1 U2 ( .A(mode[0]), .ZN(n1) );
  NOR2_X2 U3 ( .A1(mode[1]), .A2(n1), .ZN(n39) );
  CLKBUF_X1 U4 ( .A(n39), .Z(n46) );
  NOR2_X2 U5 ( .A1(mode[1]), .A2(mode[0]), .ZN(n38) );
  AOI22_X1 U6 ( .A1(n46), .A2(wact_fft), .B1(n38), .B2(wact_istream), .ZN(n2)
         );
  INV_X1 U7 ( .A(n2), .ZN(wact_ram) );
  AOI22_X1 U8 ( .A1(n39), .A2(wdw_fft[0]), .B1(n38), .B2(wdw_istream[0]), .ZN(
        n3) );
  INV_X1 U9 ( .A(n3), .ZN(wdw_ram[0]) );
  AOI22_X1 U10 ( .A1(n46), .A2(wdw_fft[1]), .B1(n38), .B2(wdw_istream[1]), 
        .ZN(n4) );
  INV_X1 U11 ( .A(n4), .ZN(wdw_ram[1]) );
  AOI22_X1 U12 ( .A1(n39), .A2(wdw_fft[2]), .B1(n38), .B2(wdw_istream[2]), 
        .ZN(n5) );
  INV_X1 U13 ( .A(n5), .ZN(wdw_ram[2]) );
  AOI22_X1 U14 ( .A1(n46), .A2(wdw_fft[3]), .B1(n38), .B2(wdw_istream[3]), 
        .ZN(n6) );
  INV_X1 U15 ( .A(n6), .ZN(wdw_ram[3]) );
  AOI22_X1 U16 ( .A1(n39), .A2(wdw_fft[4]), .B1(n38), .B2(wdw_istream[4]), 
        .ZN(n7) );
  INV_X1 U17 ( .A(n7), .ZN(wdw_ram[4]) );
  AOI22_X1 U18 ( .A1(n46), .A2(wdw_fft[5]), .B1(n38), .B2(wdw_istream[5]), 
        .ZN(n8) );
  INV_X1 U19 ( .A(n8), .ZN(wdw_ram[5]) );
  AOI22_X1 U20 ( .A1(n39), .A2(wdw_fft[6]), .B1(n38), .B2(wdw_istream[6]), 
        .ZN(n9) );
  INV_X1 U21 ( .A(n9), .ZN(wdw_ram[6]) );
  AOI22_X1 U22 ( .A1(n39), .A2(wdw_fft[7]), .B1(n38), .B2(wdw_istream[7]), 
        .ZN(n10) );
  INV_X1 U23 ( .A(n10), .ZN(wdw_ram[7]) );
  AOI22_X1 U24 ( .A1(n39), .A2(wdw_fft[8]), .B1(n38), .B2(wdw_istream[8]), 
        .ZN(n11) );
  INV_X1 U25 ( .A(n11), .ZN(wdw_ram[8]) );
  AOI22_X1 U26 ( .A1(n39), .A2(wdw_fft[9]), .B1(n38), .B2(wdw_istream[9]), 
        .ZN(n12) );
  INV_X1 U27 ( .A(n12), .ZN(wdw_ram[9]) );
  AOI22_X1 U28 ( .A1(n39), .A2(wdw_fft[10]), .B1(n38), .B2(wdw_istream[10]), 
        .ZN(n13) );
  INV_X1 U29 ( .A(n13), .ZN(wdw_ram[10]) );
  CLKBUF_X1 U30 ( .A(n38), .Z(n33) );
  AOI22_X1 U31 ( .A1(n39), .A2(wdw_fft[11]), .B1(n33), .B2(wdw_istream[11]), 
        .ZN(n14) );
  INV_X1 U32 ( .A(n14), .ZN(wdw_ram[11]) );
  AOI22_X1 U33 ( .A1(n39), .A2(wdw_fft[12]), .B1(n33), .B2(wdw_istream[12]), 
        .ZN(n15) );
  INV_X1 U34 ( .A(n15), .ZN(wdw_ram[12]) );
  AOI22_X1 U35 ( .A1(n39), .A2(wdw_fft[13]), .B1(n33), .B2(wdw_istream[13]), 
        .ZN(n16) );
  INV_X1 U36 ( .A(n16), .ZN(wdw_ram[13]) );
  AOI22_X1 U37 ( .A1(n39), .A2(wdw_fft[14]), .B1(n33), .B2(wdw_istream[14]), 
        .ZN(n17) );
  INV_X1 U38 ( .A(n17), .ZN(wdw_ram[14]) );
  AOI22_X1 U39 ( .A1(n39), .A2(wdw_fft[15]), .B1(n33), .B2(wdw_istream[15]), 
        .ZN(n18) );
  INV_X1 U40 ( .A(n18), .ZN(wdw_ram[15]) );
  AOI22_X1 U41 ( .A1(n39), .A2(wdw_fft[16]), .B1(n33), .B2(wdw_istream[16]), 
        .ZN(n19) );
  INV_X1 U42 ( .A(n19), .ZN(wdw_ram[16]) );
  AOI22_X1 U43 ( .A1(n46), .A2(wdw_fft[17]), .B1(n33), .B2(wdw_istream[17]), 
        .ZN(n20) );
  INV_X1 U44 ( .A(n20), .ZN(wdw_ram[17]) );
  AOI22_X1 U45 ( .A1(n39), .A2(wdw_fft[18]), .B1(n33), .B2(wdw_istream[18]), 
        .ZN(n21) );
  INV_X1 U46 ( .A(n21), .ZN(wdw_ram[18]) );
  AOI22_X1 U47 ( .A1(n46), .A2(wdw_fft[19]), .B1(n33), .B2(wdw_istream[19]), 
        .ZN(n22) );
  INV_X1 U48 ( .A(n22), .ZN(wdw_ram[19]) );
  AOI22_X1 U49 ( .A1(n39), .A2(wdw_fft[20]), .B1(n33), .B2(wdw_istream[20]), 
        .ZN(n23) );
  INV_X1 U50 ( .A(n23), .ZN(wdw_ram[20]) );
  AOI22_X1 U51 ( .A1(n46), .A2(wdw_fft[21]), .B1(n33), .B2(wdw_istream[21]), 
        .ZN(n24) );
  INV_X1 U52 ( .A(n24), .ZN(wdw_ram[21]) );
  AOI22_X1 U53 ( .A1(n39), .A2(wdw_fft[22]), .B1(n33), .B2(wdw_istream[22]), 
        .ZN(n25) );
  INV_X1 U54 ( .A(n25), .ZN(wdw_ram[22]) );
  AOI22_X1 U55 ( .A1(n46), .A2(wdw_fft[23]), .B1(n33), .B2(wdw_istream[23]), 
        .ZN(n26) );
  INV_X1 U56 ( .A(n26), .ZN(wdw_ram[23]) );
  AOI22_X1 U57 ( .A1(n46), .A2(wdw_fft[24]), .B1(n38), .B2(wdw_istream[24]), 
        .ZN(n27) );
  INV_X1 U58 ( .A(n27), .ZN(wdw_ram[24]) );
  AOI22_X1 U59 ( .A1(n39), .A2(wdw_fft[25]), .B1(n38), .B2(wdw_istream[25]), 
        .ZN(n28) );
  INV_X1 U60 ( .A(n28), .ZN(wdw_ram[25]) );
  AOI22_X1 U61 ( .A1(n46), .A2(wdw_fft[26]), .B1(n33), .B2(wdw_istream[26]), 
        .ZN(n29) );
  INV_X1 U62 ( .A(n29), .ZN(wdw_ram[26]) );
  AOI22_X1 U63 ( .A1(n46), .A2(wdw_fft[27]), .B1(n33), .B2(wdw_istream[27]), 
        .ZN(n30) );
  INV_X1 U64 ( .A(n30), .ZN(wdw_ram[27]) );
  AOI22_X1 U65 ( .A1(n46), .A2(wdw_fft[28]), .B1(n33), .B2(wdw_istream[28]), 
        .ZN(n31) );
  INV_X1 U66 ( .A(n31), .ZN(wdw_ram[28]) );
  AOI22_X1 U67 ( .A1(n46), .A2(wdw_fft[29]), .B1(n38), .B2(wdw_istream[29]), 
        .ZN(n32) );
  INV_X1 U68 ( .A(n32), .ZN(wdw_ram[29]) );
  AOI22_X1 U69 ( .A1(n46), .A2(wdw_fft[30]), .B1(n33), .B2(wdw_istream[30]), 
        .ZN(n34) );
  INV_X1 U70 ( .A(n34), .ZN(wdw_ram[30]) );
  AOI22_X1 U71 ( .A1(n46), .A2(wdw_fft[31]), .B1(n33), .B2(wdw_istream[31]), 
        .ZN(n35) );
  INV_X1 U72 ( .A(n35), .ZN(wdw_ram[31]) );
  AOI22_X1 U73 ( .A1(n39), .A2(wa_fft[0]), .B1(n38), .B2(wa_istream[0]), .ZN(
        n36) );
  INV_X1 U74 ( .A(n36), .ZN(wa_ram[0]) );
  AOI22_X1 U75 ( .A1(n46), .A2(wa_fft[1]), .B1(n38), .B2(wa_istream[1]), .ZN(
        n37) );
  INV_X1 U76 ( .A(n37), .ZN(wa_ram[1]) );
  AOI22_X1 U77 ( .A1(n39), .A2(wa_fft[2]), .B1(n38), .B2(wa_istream[2]), .ZN(
        n40) );
  INV_X1 U78 ( .A(n40), .ZN(wa_ram[2]) );
  AOI22_X1 U79 ( .A1(n46), .A2(wa_fft[3]), .B1(n33), .B2(wa_istream[3]), .ZN(
        n41) );
  INV_X1 U80 ( .A(n41), .ZN(wa_ram[3]) );
  AOI22_X1 U81 ( .A1(n46), .A2(wa_fft[4]), .B1(n38), .B2(wa_istream[4]), .ZN(
        n42) );
  INV_X1 U82 ( .A(n42), .ZN(wa_ram[4]) );
  AOI22_X1 U83 ( .A1(n46), .A2(wa_fft[5]), .B1(n33), .B2(wa_istream[5]), .ZN(
        n43) );
  INV_X1 U84 ( .A(n43), .ZN(wa_ram[5]) );
  AOI22_X1 U85 ( .A1(n46), .A2(wa_fft[6]), .B1(n38), .B2(wa_istream[6]), .ZN(
        n44) );
  INV_X1 U86 ( .A(n44), .ZN(wa_ram[6]) );
  AOI22_X1 U87 ( .A1(n46), .A2(wa_fft[7]), .B1(n33), .B2(wa_istream[7]), .ZN(
        n45) );
  INV_X1 U88 ( .A(n45), .ZN(wa_ram[7]) );
  AOI22_X1 U89 ( .A1(n46), .A2(wa_fft[8]), .B1(n33), .B2(wa_istream[8]), .ZN(
        n47) );
  INV_X1 U90 ( .A(n47), .ZN(wa_ram[8]) );
endmodule



    module readBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_0 ( 
        mode, ract_fft, ra_fft, rdr_fft, ract_dma, ra_dma, rdr_dma, ract_ram, 
        ra_ram, rdr_ram );
  input [1:0] mode;
  input [8:0] ra_fft;
  output [31:0] rdr_fft;
  input [8:0] ra_dma;
  output [31:0] rdr_dma;
  output [8:0] ra_ram;
  input [31:0] rdr_ram;
  input ract_fft, ract_dma;
  output ract_ram;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13;
  tri   ract_fft;
  tri   [31:0] rdr_ram;
  tri   ract_ram;
  tri   \ra_ram[8] ;
  tri   \ra_ram[7] ;
  tri   \ra_ram[6] ;
  tri   \ra_ram[5] ;
  tri   \ra_ram[4] ;
  tri   \ra_ram[3] ;
  tri   \ra_ram[2] ;
  tri   \ra_ram[1] ;
  tri   \ra_ram[0] ;
  assign rdr_dma[31] = rdr_ram[31];
  assign rdr_fft[31] = rdr_ram[31];
  assign rdr_dma[30] = rdr_ram[30];
  assign rdr_fft[30] = rdr_ram[30];
  assign rdr_dma[29] = rdr_ram[29];
  assign rdr_fft[29] = rdr_ram[29];
  assign rdr_dma[28] = rdr_ram[28];
  assign rdr_fft[28] = rdr_ram[28];
  assign rdr_dma[27] = rdr_ram[27];
  assign rdr_fft[27] = rdr_ram[27];
  assign rdr_dma[26] = rdr_ram[26];
  assign rdr_fft[26] = rdr_ram[26];
  assign rdr_dma[25] = rdr_ram[25];
  assign rdr_fft[25] = rdr_ram[25];
  assign rdr_dma[24] = rdr_ram[24];
  assign rdr_fft[24] = rdr_ram[24];
  assign rdr_dma[23] = rdr_ram[23];
  assign rdr_fft[23] = rdr_ram[23];
  assign rdr_dma[22] = rdr_ram[22];
  assign rdr_fft[22] = rdr_ram[22];
  assign rdr_dma[21] = rdr_ram[21];
  assign rdr_fft[21] = rdr_ram[21];
  assign rdr_dma[20] = rdr_ram[20];
  assign rdr_fft[20] = rdr_ram[20];
  assign rdr_dma[19] = rdr_ram[19];
  assign rdr_fft[19] = rdr_ram[19];
  assign rdr_dma[18] = rdr_ram[18];
  assign rdr_fft[18] = rdr_ram[18];
  assign rdr_dma[17] = rdr_ram[17];
  assign rdr_fft[17] = rdr_ram[17];
  assign rdr_dma[16] = rdr_ram[16];
  assign rdr_fft[16] = rdr_ram[16];
  assign rdr_dma[15] = rdr_ram[15];
  assign rdr_fft[15] = rdr_ram[15];
  assign rdr_dma[14] = rdr_ram[14];
  assign rdr_fft[14] = rdr_ram[14];
  assign rdr_dma[13] = rdr_ram[13];
  assign rdr_fft[13] = rdr_ram[13];
  assign rdr_dma[12] = rdr_ram[12];
  assign rdr_fft[12] = rdr_ram[12];
  assign rdr_dma[11] = rdr_ram[11];
  assign rdr_fft[11] = rdr_ram[11];
  assign rdr_dma[10] = rdr_ram[10];
  assign rdr_fft[10] = rdr_ram[10];
  assign rdr_dma[9] = rdr_ram[9];
  assign rdr_fft[9] = rdr_ram[9];
  assign rdr_dma[8] = rdr_ram[8];
  assign rdr_fft[8] = rdr_ram[8];
  assign rdr_dma[7] = rdr_ram[7];
  assign rdr_fft[7] = rdr_ram[7];
  assign rdr_dma[6] = rdr_ram[6];
  assign rdr_fft[6] = rdr_ram[6];
  assign rdr_dma[5] = rdr_ram[5];
  assign rdr_fft[5] = rdr_ram[5];
  assign rdr_dma[4] = rdr_ram[4];
  assign rdr_fft[4] = rdr_ram[4];
  assign rdr_dma[3] = rdr_ram[3];
  assign rdr_fft[3] = rdr_ram[3];
  assign rdr_dma[2] = rdr_ram[2];
  assign rdr_fft[2] = rdr_ram[2];
  assign rdr_dma[1] = rdr_ram[1];
  assign rdr_fft[1] = rdr_ram[1];
  assign rdr_dma[0] = rdr_ram[0];
  assign rdr_fft[0] = rdr_ram[0];

  INV_X1 U2 ( .A(mode[1]), .ZN(n1) );
  AND2_X1 U3 ( .A1(n1), .A2(mode[0]), .ZN(n12) );
  NOR2_X1 U4 ( .A1(mode[0]), .A2(n1), .ZN(n11) );
  AOI22_X1 U5 ( .A1(n12), .A2(ract_fft), .B1(n11), .B2(ract_dma), .ZN(n2) );
  INV_X1 U6 ( .A(n2), .ZN(ract_ram) );
  AOI22_X1 U7 ( .A1(n12), .A2(ra_fft[8]), .B1(n11), .B2(ra_dma[8]), .ZN(n3) );
  INV_X1 U8 ( .A(n3), .ZN(ra_ram[8]) );
  AOI22_X1 U9 ( .A1(n12), .A2(ra_fft[7]), .B1(n11), .B2(ra_dma[7]), .ZN(n4) );
  INV_X1 U10 ( .A(n4), .ZN(ra_ram[7]) );
  AOI22_X1 U11 ( .A1(n12), .A2(ra_fft[6]), .B1(n11), .B2(ra_dma[6]), .ZN(n5)
         );
  INV_X1 U12 ( .A(n5), .ZN(ra_ram[6]) );
  AOI22_X1 U13 ( .A1(n12), .A2(ra_fft[5]), .B1(n11), .B2(ra_dma[5]), .ZN(n6)
         );
  INV_X1 U14 ( .A(n6), .ZN(ra_ram[5]) );
  AOI22_X1 U15 ( .A1(n12), .A2(ra_fft[4]), .B1(n11), .B2(ra_dma[4]), .ZN(n7)
         );
  INV_X1 U16 ( .A(n7), .ZN(ra_ram[4]) );
  AOI22_X1 U17 ( .A1(n12), .A2(ra_fft[3]), .B1(n11), .B2(ra_dma[3]), .ZN(n8)
         );
  INV_X1 U18 ( .A(n8), .ZN(ra_ram[3]) );
  AOI22_X1 U19 ( .A1(n12), .A2(ra_fft[2]), .B1(n11), .B2(ra_dma[2]), .ZN(n9)
         );
  INV_X1 U20 ( .A(n9), .ZN(ra_ram[2]) );
  AOI22_X1 U21 ( .A1(n12), .A2(ra_fft[1]), .B1(n11), .B2(ra_dma[1]), .ZN(n10)
         );
  INV_X1 U22 ( .A(n10), .ZN(ra_ram[1]) );
  AOI22_X1 U23 ( .A1(n12), .A2(ra_fft[0]), .B1(n11), .B2(ra_dma[0]), .ZN(n13)
         );
  INV_X1 U24 ( .A(n13), .ZN(ra_ram[0]) );
endmodule


module R2FFT_FFT_LENGTH1024_FFT_DW16_PL_DEPTH3 ( clk, rst, autorun, run, fin, 
        ifft, done, status, bfpexp, sact_istream, sdw_istream_real, 
        sdw_istream_imag, dmaact, dmaa, dmadr_real, dmadr_imag, twact, twa, 
        twdr_cos, ract_ram0, ra_ram0, rdr_ram0, wact_ram0, wa_ram0, wdw_ram0, 
        ract_ram1, ra_ram1, rdr_ram1, wact_ram1, wa_ram1, wdw_ram1 );
  output [2:0] status;
  output [7:0] bfpexp;
  input [15:0] sdw_istream_real;
  input [15:0] sdw_istream_imag;
  input [9:0] dmaa;
  output [15:0] dmadr_real;
  output [15:0] dmadr_imag;
  output [7:0] twa;
  input [15:0] twdr_cos;
  output [8:0] ra_ram0;
  input [31:0] rdr_ram0;
  output [8:0] wa_ram0;
  output [31:0] wdw_ram0;
  output [8:0] ra_ram1;
  input [31:0] rdr_ram1;
  output [8:0] wa_ram1;
  output [31:0] wdw_ram1;
  input clk, rst, autorun, run, fin, ifft, sact_istream, dmaact;
  output done, twact, ract_ram0, wact_ram0, ract_ram1, wact_ram1;
  wire   done, streamBufferFull, N55, N56, N57, _3_net_, _4_net_, _6_net_,
         iteratorDone, oactFftUnit, N156, N157, N158, wact_fft0, wact_fft1,
         dmaa_lsb, _9_net_, _10_net_, _11_net_, _13_net_, N217, n42, n43, n44,
         n45, n46, n47, n48, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12,
         n13, n14, n15, n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26,
         n27, n28, n29, n30, n31, n32, n33, n34, n35, n36, n37, n38, n39;
  wire   [1:0] ramAccessMode;
  wire   [9:0] istreamAddr;
  wire   [4:0] istreamBw;
  wire   [4:0] istreamMaxBw;
  wire   [31:0] sb_state_f;
  wire   [3:0] fftStageCount;
  wire   [4:0] nextBfpBw;
  wire   [4:0] currentBfpBw;
  wire   [1:0] ictrlFftUnit;
  wire   [8:0] MemAddr;
  wire   [8:0] ra_fft0;
  wire   [8:0] wa_fft0;
  wire   [31:0] wdw_fft0;
  wire   [8:0] ra_fft1;
  wire   [8:0] wa_fft1;
  wire   [31:0] wdw_fft1;
  tri   clk;
  tri   rst;
  tri   ifft;
  tri   twact;
  tri   [7:0] twa;
  tri   [15:0] twdr_cos;
  tri   ract_ram0;
  tri   [8:0] ra_ram0;
  tri   [31:0] rdr_ram0;
  tri   wact_ram0;
  tri   [8:0] wa_ram0;
  tri   [31:0] wdw_ram0;
  tri   ract_ram1;
  tri   [8:0] ra_ram1;
  tri   [31:0] rdr_ram1;
  tri   wact_ram1;
  tri   [8:0] wa_ram1;
  tri   [31:0] wdw_ram1;
  tri   iactFftUnit;
  tri   iEvenOdd;
  tri   [8:0] twiddleFactorAddr;
  tri   ract_fft0;
  tri   [31:0] rdr_fft0;
  tri   ract_fft1;
  tri   [31:0] rdr_fft1;
  tri   [31:0] rdr_dma0;
  tri   [31:0] rdr_dma1;
  assign status[2] = done;

  bitReverseCounter_BIT_WIDTH10 ubitReverseCounter ( .rst(rst), .clk(clk), 
        .clr(N217), .inc(sact_istream), .iter(istreamAddr), .countFull(
        streamBufferFull) );
  bfp_bitWidthDetector_FFT_BFPDW5_FFT_DW16_1 uistreamBitWidthDetector ( 
        .operand0(sdw_istream_real), .operand1(sdw_istream_imag), .operand2({
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0}), .operand3({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), 
        .bw(istreamBw) );
  bfp_maxBitWidth_FFT_BFPDW5_1 ubfp_maxBitWidthIstream ( .rst(rst), .clk(clk), 
        .clr(_3_net_), .bw_act(_4_net_), .bw(istreamBw), .max_bw(istreamMaxBw)
         );
  bfp_bitWidthAcc_FFT_BFPDW5_FFT_DW16 ubfpacc ( .clk(clk), .rst(rst), .init(
        n48), .bw_init(istreamMaxBw), .update(_6_net_), .bw_new(nextBfpBw), 
        .bfp_bw(currentBfpBw), .bfp_exponent(bfpexp) );
  fftAddressGenerator_FFT_N10_STAGE_COUNT_BW4 ufftAddressGenerator ( .clk(clk), 
        .rst(rst), .stageCount(fftStageCount), .run(n47), .done(iteratorDone), 
        .act(iactFftUnit), .ctrl(ictrlFftUnit), .evenOdd(iEvenOdd), .MemAddr(
        MemAddr), .twiddleFactorAddr(twiddleFactorAddr) );
  butterflyUnit_FFT_N10_FFT_DW16_FFT_BFPDW5_PL_DEPTH3 ubutterflyUnit ( .clk(
        clk), .rst(rst), .clr_bfp(n46), .ibfp(currentBfpBw), .obfp(nextBfpBw), 
        .iact(iactFftUnit), .oact(oactFftUnit), .ictrl(ictrlFftUnit), 
        .MemAddr(MemAddr), .twiddleFactorAddr(twiddleFactorAddr), .evenOdd(
        iEvenOdd), .ifft(ifft), .twact(twact), .twa(twa), .twdr_cos(twdr_cos), 
        .ract_ram0(ract_fft0), .ra_ram0(ra_fft0), .rdr_ram0(rdr_fft0), 
        .wact_ram0(wact_fft0), .wa_ram0(wa_fft0), .wdw_ram0(wdw_fft0), 
        .ract_ram1(ract_fft1), .ra_ram1(ra_fft1), .rdr_ram1(rdr_fft1), 
        .wact_ram1(wact_fft1), .wa_ram1(wa_fft1), .wdw_ram1(wdw_fft1) );
  readBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_1 readBusMuxEven ( 
        .mode(ramAccessMode), .ract_fft(ract_fft0), .ra_fft(ra_fft0), 
        .rdr_fft(rdr_fft0), .ract_dma(_9_net_), .ra_dma(dmaa[9:1]), .rdr_dma(
        rdr_dma0), .ract_ram(ract_ram0), .ra_ram(ra_ram0), .rdr_ram(rdr_ram0)
         );
  readBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_0 readBusMuxOdd ( 
        .mode(ramAccessMode), .ract_fft(ract_fft1), .ra_fft(ra_fft1), 
        .rdr_fft(rdr_fft1), .ract_dma(_10_net_), .ra_dma(dmaa[9:1]), .rdr_dma(
        rdr_dma1), .ract_ram(ract_ram1), .ra_ram(ra_ram1), .rdr_ram(rdr_ram1)
         );
  writeBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_1 writeBusMuxEven ( 
        .mode(ramAccessMode), .wact_fft(wact_fft0), .wa_fft(wa_fft0), 
        .wdw_fft(wdw_fft0), .wact_istream(_11_net_), .wa_istream(
        istreamAddr[9:1]), .wdw_istream({sdw_istream_imag, sdw_istream_real}), 
        .wact_ram(wact_ram0), .wa_ram(wa_ram0), .wdw_ram(wdw_ram0) );
  writeBusMux_FFT_N10_FFT_DW16_MODE_INPUT_STREAM0_MODE_RUN_FFT1_MODE_DMA2_MODE_DISABLE3_0 writeBusMuxOdd ( 
        .mode(ramAccessMode), .wact_fft(wact_fft1), .wa_fft(wa_fft1), 
        .wdw_fft(wdw_fft1), .wact_istream(_13_net_), .wa_istream(
        istreamAddr[9:1]), .wdw_istream({sdw_istream_imag, sdw_istream_real}), 
        .wact_ram(wact_ram1), .wa_ram(wa_ram1), .wdw_ram(wdw_ram1) );
  DFF_X1 \fftStageCount_reg[1]  ( .D(n42), .CK(clk), .Q(fftStageCount[1]), 
        .QN(n37) );
  DFF_X1 \sb_state_f_reg[2]  ( .D(N158), .CK(clk), .Q(sb_state_f[2]), .QN(n36)
         );
  DFF_X1 \sb_state_f_reg[1]  ( .D(N157), .CK(clk), .Q(sb_state_f[1]), .QN(n39)
         );
  DFF_X1 \fftStageCount_reg[2]  ( .D(n43), .CK(clk), .Q(fftStageCount[2]), 
        .QN(n35) );
  DFF_X1 \fftStageCount_reg[3]  ( .D(n44), .CK(clk), .Q(fftStageCount[3]) );
  DFF_X1 \sb_state_f_reg[0]  ( .D(N156), .CK(clk), .Q(sb_state_f[0]), .QN(n33)
         );
  DFF_X1 \status_f_reg[2]  ( .D(N57), .CK(clk), .Q(done), .QN(n32) );
  DFF_X1 \status_f_reg[0]  ( .D(N55), .CK(clk), .Q(status[0]), .QN(n38) );
  DFF_X1 \status_f_reg[1]  ( .D(N56), .CK(clk), .Q(status[1]), .QN(n34) );
  DFF_X1 \fftStageCount_reg[0]  ( .D(n45), .CK(clk), .Q(fftStageCount[0]) );
  DFF_X2 dmaa_lsb_reg ( .D(dmaa[0]), .CK(clk), .Q(dmaa_lsb) );
  NOR3_X2 U3 ( .A1(sb_state_f[1]), .A2(sb_state_f[2]), .A3(n33), .ZN(n48) );
  OAI221_X1 U4 ( .B1(n12), .B2(done), .C1(n12), .C2(n38), .A(n34), .ZN(
        ramAccessMode[0]) );
  NAND2_X1 U5 ( .A1(status[0]), .A2(n32), .ZN(ramAccessMode[1]) );
  INV_X1 U6 ( .A(ramAccessMode[1]), .ZN(n12) );
  AND2_X1 U7 ( .A1(dmaa[0]), .A2(dmaact), .ZN(_10_net_) );
  NOR2_X1 U8 ( .A1(n36), .A2(sb_state_f[1]), .ZN(n13) );
  NAND2_X1 U9 ( .A1(n33), .A2(n13), .ZN(n5) );
  INV_X1 U10 ( .A(n5), .ZN(n46) );
  INV_X1 U11 ( .A(rst), .ZN(n22) );
  OAI21_X1 U12 ( .B1(sb_state_f[1]), .B2(sb_state_f[2]), .A(n22), .ZN(n29) );
  OAI21_X1 U13 ( .B1(n29), .B2(n46), .A(fftStageCount[0]), .ZN(n31) );
  OAI211_X1 U14 ( .C1(n46), .C2(fftStageCount[0]), .A(n22), .B(n31), .ZN(n1)
         );
  INV_X1 U15 ( .A(n1), .ZN(n45) );
  AND2_X1 U16 ( .A1(sact_istream), .A2(istreamAddr[0]), .ZN(_13_net_) );
  INV_X1 U17 ( .A(dmaact), .ZN(n2) );
  NOR2_X1 U18 ( .A1(dmaa[0]), .A2(n2), .ZN(_9_net_) );
  INV_X1 U19 ( .A(sact_istream), .ZN(n18) );
  NOR2_X1 U20 ( .A1(istreamAddr[0]), .A2(n18), .ZN(_11_net_) );
  MUX2_X1 U21 ( .A(rdr_dma0[0]), .B(rdr_dma1[0]), .S(dmaa_lsb), .Z(
        dmadr_real[0]) );
  MUX2_X1 U22 ( .A(rdr_dma0[1]), .B(rdr_dma1[1]), .S(dmaa_lsb), .Z(
        dmadr_real[1]) );
  MUX2_X1 U23 ( .A(rdr_dma0[2]), .B(rdr_dma1[2]), .S(dmaa_lsb), .Z(
        dmadr_real[2]) );
  MUX2_X1 U24 ( .A(rdr_dma0[3]), .B(rdr_dma1[3]), .S(dmaa_lsb), .Z(
        dmadr_real[3]) );
  MUX2_X1 U25 ( .A(rdr_dma0[4]), .B(rdr_dma1[4]), .S(dmaa_lsb), .Z(
        dmadr_real[4]) );
  MUX2_X1 U26 ( .A(rdr_dma0[5]), .B(rdr_dma1[5]), .S(dmaa_lsb), .Z(
        dmadr_real[5]) );
  MUX2_X1 U27 ( .A(rdr_dma0[6]), .B(rdr_dma1[6]), .S(dmaa_lsb), .Z(
        dmadr_real[6]) );
  MUX2_X1 U28 ( .A(rdr_dma0[7]), .B(rdr_dma1[7]), .S(dmaa_lsb), .Z(
        dmadr_real[7]) );
  MUX2_X1 U29 ( .A(rdr_dma0[8]), .B(rdr_dma1[8]), .S(dmaa_lsb), .Z(
        dmadr_real[8]) );
  MUX2_X1 U30 ( .A(rdr_dma0[9]), .B(rdr_dma1[9]), .S(dmaa_lsb), .Z(
        dmadr_real[9]) );
  MUX2_X1 U31 ( .A(rdr_dma0[10]), .B(rdr_dma1[10]), .S(dmaa_lsb), .Z(
        dmadr_real[10]) );
  MUX2_X1 U32 ( .A(rdr_dma0[11]), .B(rdr_dma1[11]), .S(dmaa_lsb), .Z(
        dmadr_real[11]) );
  MUX2_X1 U33 ( .A(rdr_dma0[12]), .B(rdr_dma1[12]), .S(dmaa_lsb), .Z(
        dmadr_real[12]) );
  MUX2_X1 U34 ( .A(rdr_dma0[13]), .B(rdr_dma1[13]), .S(dmaa_lsb), .Z(
        dmadr_real[13]) );
  MUX2_X1 U35 ( .A(rdr_dma0[14]), .B(rdr_dma1[14]), .S(dmaa_lsb), .Z(
        dmadr_real[14]) );
  MUX2_X1 U36 ( .A(rdr_dma0[15]), .B(rdr_dma1[15]), .S(dmaa_lsb), .Z(
        dmadr_real[15]) );
  MUX2_X1 U37 ( .A(rdr_dma0[16]), .B(rdr_dma1[16]), .S(dmaa_lsb), .Z(
        dmadr_imag[0]) );
  MUX2_X1 U38 ( .A(rdr_dma0[17]), .B(rdr_dma1[17]), .S(dmaa_lsb), .Z(
        dmadr_imag[1]) );
  MUX2_X1 U39 ( .A(rdr_dma0[18]), .B(rdr_dma1[18]), .S(dmaa_lsb), .Z(
        dmadr_imag[2]) );
  MUX2_X1 U40 ( .A(rdr_dma0[19]), .B(rdr_dma1[19]), .S(dmaa_lsb), .Z(
        dmadr_imag[3]) );
  MUX2_X1 U41 ( .A(rdr_dma0[20]), .B(rdr_dma1[20]), .S(dmaa_lsb), .Z(
        dmadr_imag[4]) );
  MUX2_X1 U42 ( .A(rdr_dma0[21]), .B(rdr_dma1[21]), .S(dmaa_lsb), .Z(
        dmadr_imag[5]) );
  MUX2_X1 U43 ( .A(rdr_dma0[22]), .B(rdr_dma1[22]), .S(dmaa_lsb), .Z(
        dmadr_imag[6]) );
  MUX2_X1 U44 ( .A(rdr_dma0[23]), .B(rdr_dma1[23]), .S(dmaa_lsb), .Z(
        dmadr_imag[7]) );
  MUX2_X1 U45 ( .A(rdr_dma0[24]), .B(rdr_dma1[24]), .S(dmaa_lsb), .Z(
        dmadr_imag[8]) );
  MUX2_X1 U46 ( .A(rdr_dma0[25]), .B(rdr_dma1[25]), .S(dmaa_lsb), .Z(
        dmadr_imag[9]) );
  MUX2_X1 U47 ( .A(rdr_dma0[26]), .B(rdr_dma1[26]), .S(dmaa_lsb), .Z(
        dmadr_imag[10]) );
  MUX2_X1 U48 ( .A(rdr_dma0[27]), .B(rdr_dma1[27]), .S(dmaa_lsb), .Z(
        dmadr_imag[11]) );
  MUX2_X1 U49 ( .A(rdr_dma0[28]), .B(rdr_dma1[28]), .S(dmaa_lsb), .Z(
        dmadr_imag[12]) );
  MUX2_X1 U50 ( .A(rdr_dma0[29]), .B(rdr_dma1[29]), .S(dmaa_lsb), .Z(
        dmadr_imag[13]) );
  MUX2_X1 U51 ( .A(rdr_dma0[30]), .B(rdr_dma1[30]), .S(dmaa_lsb), .Z(
        dmadr_imag[14]) );
  MUX2_X1 U52 ( .A(rdr_dma0[31]), .B(rdr_dma1[31]), .S(dmaa_lsb), .Z(
        dmadr_imag[15]) );
  NOR3_X1 U53 ( .A1(sb_state_f[0]), .A2(iteratorDone), .A3(n39), .ZN(n4) );
  AND4_X1 U54 ( .A1(n37), .A2(n35), .A3(fftStageCount[0]), .A4(
        fftStageCount[3]), .ZN(n6) );
  OAI21_X1 U55 ( .B1(sb_state_f[0]), .B2(n6), .A(n13), .ZN(n11) );
  OAI221_X1 U56 ( .B1(n33), .B2(sb_state_f[1]), .C1(n33), .C2(oactFftUnit), 
        .A(n36), .ZN(n3) );
  NAND3_X1 U57 ( .A1(n12), .A2(status[1]), .A3(n22), .ZN(n24) );
  AOI221_X1 U58 ( .B1(n4), .B2(n11), .C1(n3), .C2(n11), .A(n24), .ZN(N156) );
  NAND2_X1 U59 ( .A1(sb_state_f[1]), .A2(n36), .ZN(n9) );
  NOR2_X1 U60 ( .A1(sb_state_f[0]), .A2(n9), .ZN(n47) );
  NOR2_X1 U61 ( .A1(n6), .A2(n5), .ZN(_6_net_) );
  INV_X1 U62 ( .A(oactFftUnit), .ZN(n8) );
  NOR3_X1 U63 ( .A1(n48), .A2(n47), .A3(_6_net_), .ZN(n7) );
  AOI221_X1 U64 ( .B1(n8), .B2(n7), .C1(n9), .C2(n7), .A(n24), .ZN(N157) );
  OR3_X1 U65 ( .A1(n33), .A2(n9), .A3(oactFftUnit), .ZN(n10) );
  AOI21_X1 U66 ( .B1(n11), .B2(n10), .A(n24), .ZN(N158) );
  NOR2_X1 U67 ( .A1(status[0]), .A2(status[1]), .ZN(_3_net_) );
  NAND2_X1 U68 ( .A1(n12), .A2(n34), .ZN(N217) );
  NAND2_X1 U69 ( .A1(sb_state_f[0]), .A2(n13), .ZN(n23) );
  INV_X1 U70 ( .A(n23), .ZN(n19) );
  NOR2_X1 U71 ( .A1(autorun), .A2(n18), .ZN(n15) );
  OAI211_X1 U72 ( .C1(run), .C2(n34), .A(n32), .B(n38), .ZN(n14) );
  OAI221_X1 U73 ( .B1(N217), .B2(streamBufferFull), .C1(N217), .C2(n15), .A(
        n14), .ZN(n16) );
  AOI21_X1 U74 ( .B1(_3_net_), .B2(fin), .A(n16), .ZN(n17) );
  OAI22_X1 U75 ( .A1(n19), .A2(n24), .B1(rst), .B2(n17), .ZN(N55) );
  NOR2_X1 U76 ( .A1(N217), .A2(n18), .ZN(_4_net_) );
  AOI211_X1 U77 ( .C1(status[0]), .C2(n19), .A(done), .B(n34), .ZN(n20) );
  AOI21_X1 U78 ( .B1(_4_net_), .B2(streamBufferFull), .A(n20), .ZN(n21) );
  NOR2_X1 U79 ( .A1(rst), .A2(n21), .ZN(N56) );
  NAND3_X1 U80 ( .A1(done), .A2(_3_net_), .A3(n22), .ZN(n25) );
  OAI22_X1 U81 ( .A1(fin), .A2(n25), .B1(n24), .B2(n23), .ZN(N57) );
  NOR2_X1 U83 ( .A1(n37), .A2(n31), .ZN(n30) );
  INV_X1 U84 ( .A(n30), .ZN(n28) );
  NOR2_X1 U85 ( .A1(n28), .A2(n35), .ZN(n27) );
  NOR2_X1 U86 ( .A1(n27), .A2(fftStageCount[3]), .ZN(n26) );
  AOI211_X1 U87 ( .C1(n27), .C2(fftStageCount[3]), .A(n29), .B(n26), .ZN(n44)
         );
  AOI211_X1 U88 ( .C1(n28), .C2(n35), .A(n27), .B(n29), .ZN(n43) );
  AOI211_X1 U89 ( .C1(n37), .C2(n31), .A(n30), .B(n29), .ZN(n42) );
endmodule


module r2fft_top ( clk, rst, autorun, run, fin, ifft, done, status, bfpexp, 
        sact_istream, sdw_istream_real, sdw_istream_imag, dmaact, dmaa, 
        dmadr_real, dmadr_imag );
  output [2:0] status;
  output [7:0] bfpexp;
  input [15:0] sdw_istream_real;
  input [15:0] sdw_istream_imag;
  input [9:0] dmaa;
  output [15:0] dmadr_real;
  output [15:0] dmadr_imag;
  input clk, rst, autorun, run, fin, ifft, sact_istream, dmaact;
  output done;

  tri   clk;
  tri   rst;
  tri   ifft;
  tri   twact;
  tri   [7:0] twa;
  tri   [15:0] twdr_cos;
  tri   ract_ram0;
  tri   [8:0] ra_ram0;
  tri   [31:0] rdr_ram0;
  tri   wact_ram0;
  tri   [8:0] wa_ram0;
  tri   [31:0] wdw_ram0;
  tri   ract_ram1;
  tri   [8:0] ra_ram1;
  tri   [31:0] rdr_ram1;
  tri   wact_ram1;
  tri   [8:0] wa_ram1;
  tri   [31:0] wdw_ram1;

  R2FFT_FFT_LENGTH1024_FFT_DW16_PL_DEPTH3 uR2FFT ( .clk(clk), .rst(rst), 
        .autorun(autorun), .run(run), .fin(fin), .ifft(ifft), .done(done), 
        .status(status), .bfpexp(bfpexp), .sact_istream(sact_istream), 
        .sdw_istream_real(sdw_istream_real), .sdw_istream_imag(
        sdw_istream_imag), .dmaact(dmaact), .dmaa(dmaa), .dmadr_real(
        dmadr_real), .dmadr_imag(dmadr_imag), .twact(twact), .twa(twa), 
        .twdr_cos(twdr_cos), .ract_ram0(ract_ram0), .ra_ram0(ra_ram0), 
        .rdr_ram0(rdr_ram0), .wact_ram0(wact_ram0), .wa_ram0(wa_ram0), 
        .wdw_ram0(wdw_ram0), .ract_ram1(ract_ram1), .ra_ram1(ra_ram1), 
        .rdr_ram1(rdr_ram1), .wact_ram1(wact_ram1), .wa_ram1(wa_ram1), 
        .wdw_ram1(wdw_ram1) );
  twrom utwrom ( .clk(clk), .twact(twact), .twa(twa), .twdr_cos(twdr_cos) );
  dpram ram0 ( .clk(clk), .ract(ract_ram0), .ra(ra_ram0), .rdr(rdr_ram0), 
        .wact(wact_ram0), .wa(wa_ram0), .wdw(wdw_ram0) );
  dpram ram1 ( .clk(clk), .ract(ract_ram1), .ra(ra_ram1), .rdr(rdr_ram1), 
        .wact(wact_ram1), .wa(wa_ram1), .wdw(wdw_ram1) );
endmodule

