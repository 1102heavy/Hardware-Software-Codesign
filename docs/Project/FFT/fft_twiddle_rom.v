// ─────────────────────────────────────────────────────────────────────────
// fft_twiddle_rom.v
//
// Combinational ROM: 512 × 32 bits.
// Word format: bits[31:16] = w_real (Q1.15), bits[15:0] = w_imag (Q1.15)
// W_1024^k = cos(2πk/1024) - j*sin(2πk/1024) stored at address k.
// Read latency: 0 clock cycles (combinational, inferred as distributed RAM).
//
// Twiddle factors are hardcoded — no external .mem file required.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_twiddle_rom #(
    parameter DEPTH = 512,
    parameter ABITS = 9      // ceil(log2(512)) = 9
)(
    input  wire              clk,   // unused; kept for interface compatibility
    input  wire [ABITS-1:0]  addr,
    output wire signed [15:0] w_real,
    output wire signed [15:0] w_imag
);

    (* rom_style = "distributed" *) reg [31:0] rom [0:DEPTH-1];

    initial begin
        rom[  0] = 32'h7fff0000; rom[  1] = 32'h7fffff37; rom[  2] = 32'h7ffefe6e; rom[  3] = 32'h7ffafda5;
        rom[  4] = 32'h7ff6fcdc; rom[  5] = 32'h7ff1fc13; rom[  6] = 32'h7feafb4a; rom[  7] = 32'h7fe2fa81;
        rom[  8] = 32'h7fd9f9b8; rom[  9] = 32'h7fcef8ef; rom[ 10] = 32'h7fc2f827; rom[ 11] = 32'h7fb5f75e;
        rom[ 12] = 32'h7fa7f695; rom[ 13] = 32'h7f98f5cd; rom[ 14] = 32'h7f87f505; rom[ 15] = 32'h7f75f43c;
        rom[ 16] = 32'h7f62f374; rom[ 17] = 32'h7f4ef2ac; rom[ 18] = 32'h7f38f1e4; rom[ 19] = 32'h7f22f11c;
        rom[ 20] = 32'h7f0af055; rom[ 21] = 32'h7ef0ef8d; rom[ 22] = 32'h7ed6eec6; rom[ 23] = 32'h7ebaedff;
        rom[ 24] = 32'h7e9ded38; rom[ 25] = 32'h7e7fec71; rom[ 26] = 32'h7e60ebab; rom[ 27] = 32'h7e3feae4;
        rom[ 28] = 32'h7e1eea1e; rom[ 29] = 32'h7dfbe958; rom[ 30] = 32'h7dd6e892; rom[ 31] = 32'h7db1e7cd;
        rom[ 32] = 32'h7d8ae707; rom[ 33] = 32'h7d63e642; rom[ 34] = 32'h7d3ae57d; rom[ 35] = 32'h7d0fe4b9;
        rom[ 36] = 32'h7ce4e3f4; rom[ 37] = 32'h7cb7e330; rom[ 38] = 32'h7c89e26d; rom[ 39] = 32'h7c5ae1a9;
        rom[ 40] = 32'h7c2ae0e6; rom[ 41] = 32'h7bf9e023; rom[ 42] = 32'h7bc6df61; rom[ 43] = 32'h7b92de9e;
        rom[ 44] = 32'h7b5ddddc; rom[ 45] = 32'h7b27dd1b; rom[ 46] = 32'h7aefdc59; rom[ 47] = 32'h7ab7db99;
        rom[ 48] = 32'h7a7ddad8; rom[ 49] = 32'h7a42da18; rom[ 50] = 32'h7a06d958; rom[ 51] = 32'h79c9d898;
        rom[ 52] = 32'h798ad7d9; rom[ 53] = 32'h794ad71b; rom[ 54] = 32'h790ad65c; rom[ 55] = 32'h78c8d59e;
        rom[ 56] = 32'h7885d4e1; rom[ 57] = 32'h7840d424; rom[ 58] = 32'h77fbd367; rom[ 59] = 32'h77b4d2ab;
        rom[ 60] = 32'h776cd1ef; rom[ 61] = 32'h7723d134; rom[ 62] = 32'h76d9d079; rom[ 63] = 32'h768ecfbe;
        rom[ 64] = 32'h7642cf04; rom[ 65] = 32'h75f4ce4b; rom[ 66] = 32'h75a6cd92; rom[ 67] = 32'h7556ccd9;
        rom[ 68] = 32'h7505cc21; rom[ 69] = 32'h74b3cb69; rom[ 70] = 32'h7460cab2; rom[ 71] = 32'h740bc9fc;
        rom[ 72] = 32'h73b6c946; rom[ 73] = 32'h735fc890; rom[ 74] = 32'h7308c7db; rom[ 75] = 32'h72afc727;
        rom[ 76] = 32'h7255c673; rom[ 77] = 32'h71fac5c0; rom[ 78] = 32'h719ec50d; rom[ 79] = 32'h7141c45b;
        rom[ 80] = 32'h70e3c3a9; rom[ 81] = 32'h7083c2f8; rom[ 82] = 32'h7023c248; rom[ 83] = 32'h6fc2c198;
        rom[ 84] = 32'h6f5fc0e9; rom[ 85] = 32'h6efbc03a; rom[ 86] = 32'h6e97bf8c; rom[ 87] = 32'h6e31bedf;
        rom[ 88] = 32'h6dcabe32; rom[ 89] = 32'h6d62bd86; rom[ 90] = 32'h6cf9bcda; rom[ 91] = 32'h6c8fbc2f;
        rom[ 92] = 32'h6c24bb85; rom[ 93] = 32'h6bb8badc; rom[ 94] = 32'h6b4bba33; rom[ 95] = 32'h6addb98b;
        rom[ 96] = 32'h6a6eb8e3; rom[ 97] = 32'h69fdb83c; rom[ 98] = 32'h698cb796; rom[ 99] = 32'h691ab6f1;
        rom[100] = 32'h68a7b64c; rom[101] = 32'h6832b5a8; rom[102] = 32'h67bdb505; rom[103] = 32'h6747b462;
        rom[104] = 32'h66d0b3c0; rom[105] = 32'h6657b31f; rom[106] = 32'h65deb27f; rom[107] = 32'h6564b1df;
        rom[108] = 32'h64e9b140; rom[109] = 32'h646cb0a2; rom[110] = 32'h63efb005; rom[111] = 32'h6371af68;
        rom[112] = 32'h62f2aecc; rom[113] = 32'h6272ae31; rom[114] = 32'h61f1ad97; rom[115] = 32'h616facfd;
        rom[116] = 32'h60ecac65; rom[117] = 32'h6068abcd; rom[118] = 32'h5fe4ab36; rom[119] = 32'h5f5eaaa0;
        rom[120] = 32'h5ed7aa0a; rom[121] = 32'h5e50a976; rom[122] = 32'h5dc8a8e2; rom[123] = 32'h5d3ea84f;
        rom[124] = 32'h5cb4a7bd; rom[125] = 32'h5c29a72c; rom[126] = 32'h5b9da69c; rom[127] = 32'h5b10a60c;
        rom[128] = 32'h5a82a57e; rom[129] = 32'h59f4a4f0; rom[130] = 32'h5964a463; rom[131] = 32'h58d4a3d7;
        rom[132] = 32'h5843a34c; rom[133] = 32'h57b1a2c2; rom[134] = 32'h571ea238; rom[135] = 32'h568aa1b0;
        rom[136] = 32'h55f6a129; rom[137] = 32'h5560a0a2; rom[138] = 32'h54caa01c; rom[139] = 32'h54339f98;
        rom[140] = 32'h539b9f14; rom[141] = 32'h53039e91; rom[142] = 32'h52699e0f; rom[143] = 32'h51cf9d8e;
        rom[144] = 32'h51349d0e; rom[145] = 32'h50989c8f; rom[146] = 32'h4ffb9c11; rom[147] = 32'h4f5e9b94;
        rom[148] = 32'h4ec09b17; rom[149] = 32'h4e219a9c; rom[150] = 32'h4d819a22; rom[151] = 32'h4ce199a9;
        rom[152] = 32'h4c409930; rom[153] = 32'h4b9e98b9; rom[154] = 32'h4afb9843; rom[155] = 32'h4a5897ce;
        rom[156] = 32'h49b49759; rom[157] = 32'h490f96e6; rom[158] = 32'h486a9674; rom[159] = 32'h47c49603;
        rom[160] = 32'h471d9592; rom[161] = 32'h46759523; rom[162] = 32'h45cd94b5; rom[163] = 32'h45249448;
        rom[164] = 32'h447b93dc; rom[165] = 32'h43d19371; rom[166] = 32'h43269307; rom[167] = 32'h427a929e;
        rom[168] = 32'h41ce9236; rom[169] = 32'h412191cf; rom[170] = 32'h40749169; rom[171] = 32'h3fc69105;
        rom[172] = 32'h3f1790a1; rom[173] = 32'h3e68903e; rom[174] = 32'h3db88fdd; rom[175] = 32'h3d088f7d;
        rom[176] = 32'h3c578f1d; rom[177] = 32'h3ba58ebf; rom[178] = 32'h3af38e62; rom[179] = 32'h3a408e06;
        rom[180] = 32'h398d8dab; rom[181] = 32'h38d98d51; rom[182] = 32'h38258cf8; rom[183] = 32'h37708ca1;
        rom[184] = 32'h36ba8c4a; rom[185] = 32'h36048bf5; rom[186] = 32'h354e8ba0; rom[187] = 32'h34978b4d;
        rom[188] = 32'h33df8afb; rom[189] = 32'h33278aaa; rom[190] = 32'h326e8a5a; rom[191] = 32'h31b58a0c;
        rom[192] = 32'h30fc89be; rom[193] = 32'h30428972; rom[194] = 32'h2f878927; rom[195] = 32'h2ecc88dd;
        rom[196] = 32'h2e118894; rom[197] = 32'h2d55884c; rom[198] = 32'h2c998805; rom[199] = 32'h2bdc87c0;
        rom[200] = 32'h2b1f877b; rom[201] = 32'h2a628738; rom[202] = 32'h29a486f6; rom[203] = 32'h28e586b6;
        rom[204] = 32'h28278676; rom[205] = 32'h27688637; rom[206] = 32'h26a885fa; rom[207] = 32'h25e885be;
        rom[208] = 32'h25288583; rom[209] = 32'h24678549; rom[210] = 32'h23a78511; rom[211] = 32'h22e584d9;
        rom[212] = 32'h222484a3; rom[213] = 32'h2162846e; rom[214] = 32'h209f843a; rom[215] = 32'h1fdd8407;
        rom[216] = 32'h1f1a83d6; rom[217] = 32'h1e5783a6; rom[218] = 32'h1d938377; rom[219] = 32'h1cd08349;
        rom[220] = 32'h1c0c831c; rom[221] = 32'h1b4782f1; rom[222] = 32'h1a8382c6; rom[223] = 32'h19be829d;
        rom[224] = 32'h18f98276; rom[225] = 32'h1833824f; rom[226] = 32'h176e822a; rom[227] = 32'h16a88205;
        rom[228] = 32'h15e281e2; rom[229] = 32'h151c81c1; rom[230] = 32'h145581a0; rom[231] = 32'h138f8181;
        rom[232] = 32'h12c88163; rom[233] = 32'h12018146; rom[234] = 32'h113a812a; rom[235] = 32'h10738110;
        rom[236] = 32'h0fab80f6; rom[237] = 32'h0ee480de; rom[238] = 32'h0e1c80c8; rom[239] = 32'h0d5480b2;
        rom[240] = 32'h0c8c809e; rom[241] = 32'h0bc4808b; rom[242] = 32'h0afb8079; rom[243] = 32'h0a338068;
        rom[244] = 32'h096b8059; rom[245] = 32'h08a2804b; rom[246] = 32'h07d9803e; rom[247] = 32'h07118032;
        rom[248] = 32'h06488027; rom[249] = 32'h057f801e; rom[250] = 32'h04b68016; rom[251] = 32'h03ed800f;
        rom[252] = 32'h0324800a; rom[253] = 32'h025b8006; rom[254] = 32'h01928002; rom[255] = 32'h00c98001;
        rom[256] = 32'h00008000; rom[257] = 32'hff378001; rom[258] = 32'hfe6e8002; rom[259] = 32'hfda58006;
        rom[260] = 32'hfcdc800a; rom[261] = 32'hfc13800f; rom[262] = 32'hfb4a8016; rom[263] = 32'hfa81801e;
        rom[264] = 32'hf9b88027; rom[265] = 32'hf8ef8032; rom[266] = 32'hf827803e; rom[267] = 32'hf75e804b;
        rom[268] = 32'hf6958059; rom[269] = 32'hf5cd8068; rom[270] = 32'hf5058079; rom[271] = 32'hf43c808b;
        rom[272] = 32'hf374809e; rom[273] = 32'hf2ac80b2; rom[274] = 32'hf1e480c8; rom[275] = 32'hf11c80de;
        rom[276] = 32'hf05580f6; rom[277] = 32'hef8d8110; rom[278] = 32'heec6812a; rom[279] = 32'hedff8146;
        rom[280] = 32'hed388163; rom[281] = 32'hec718181; rom[282] = 32'hebab81a0; rom[283] = 32'heae481c1;
        rom[284] = 32'hea1e81e2; rom[285] = 32'he9588205; rom[286] = 32'he892822a; rom[287] = 32'he7cd824f;
        rom[288] = 32'he7078276; rom[289] = 32'he642829d; rom[290] = 32'he57d82c6; rom[291] = 32'he4b982f1;
        rom[292] = 32'he3f4831c; rom[293] = 32'he3308349; rom[294] = 32'he26d8377; rom[295] = 32'he1a983a6;
        rom[296] = 32'he0e683d6; rom[297] = 32'he0238407; rom[298] = 32'hdf61843a; rom[299] = 32'hde9e846e;
        rom[300] = 32'hdddc84a3; rom[301] = 32'hdd1b84d9; rom[302] = 32'hdc598511; rom[303] = 32'hdb998549;
        rom[304] = 32'hdad88583; rom[305] = 32'hda1885be; rom[306] = 32'hd95885fa; rom[307] = 32'hd8988637;
        rom[308] = 32'hd7d98676; rom[309] = 32'hd71b86b6; rom[310] = 32'hd65c86f6; rom[311] = 32'hd59e8738;
        rom[312] = 32'hd4e1877b; rom[313] = 32'hd42487c0; rom[314] = 32'hd3678805; rom[315] = 32'hd2ab884c;
        rom[316] = 32'hd1ef8894; rom[317] = 32'hd13488dd; rom[318] = 32'hd0798927; rom[319] = 32'hcfbe8972;
        rom[320] = 32'hcf0489be; rom[321] = 32'hce4b8a0c; rom[322] = 32'hcd928a5a; rom[323] = 32'hccd98aaa;
        rom[324] = 32'hcc218afb; rom[325] = 32'hcb698b4d; rom[326] = 32'hcab28ba0; rom[327] = 32'hc9fc8bf5;
        rom[328] = 32'hc9468c4a; rom[329] = 32'hc8908ca1; rom[330] = 32'hc7db8cf8; rom[331] = 32'hc7278d51;
        rom[332] = 32'hc6738dab; rom[333] = 32'hc5c08e06; rom[334] = 32'hc50d8e62; rom[335] = 32'hc45b8ebf;
        rom[336] = 32'hc3a98f1d; rom[337] = 32'hc2f88f7d; rom[338] = 32'hc2488fdd; rom[339] = 32'hc198903e;
        rom[340] = 32'hc0e990a1; rom[341] = 32'hc03a9105; rom[342] = 32'hbf8c9169; rom[343] = 32'hbedf91cf;
        rom[344] = 32'hbe329236; rom[345] = 32'hbd86929e; rom[346] = 32'hbcda9307; rom[347] = 32'hbc2f9371;
        rom[348] = 32'hbb8593dc; rom[349] = 32'hbadc9448; rom[350] = 32'hba3394b5; rom[351] = 32'hb98b9523;
        rom[352] = 32'hb8e39592; rom[353] = 32'hb83c9603; rom[354] = 32'hb7969674; rom[355] = 32'hb6f196e6;
        rom[356] = 32'hb64c9759; rom[357] = 32'hb5a897ce; rom[358] = 32'hb5059843; rom[359] = 32'hb46298b9;
        rom[360] = 32'hb3c09930; rom[361] = 32'hb31f99a9; rom[362] = 32'hb27f9a22; rom[363] = 32'hb1df9a9c;
        rom[364] = 32'hb1409b17; rom[365] = 32'hb0a29b94; rom[366] = 32'hb0059c11; rom[367] = 32'haf689c8f;
        rom[368] = 32'haecc9d0e; rom[369] = 32'hae319d8e; rom[370] = 32'had979e0f; rom[371] = 32'hacfd9e91;
        rom[372] = 32'hac659f14; rom[373] = 32'habcd9f98; rom[374] = 32'hab36a01c; rom[375] = 32'haaa0a0a2;
        rom[376] = 32'haa0aa129; rom[377] = 32'ha976a1b0; rom[378] = 32'ha8e2a238; rom[379] = 32'ha84fa2c2;
        rom[380] = 32'ha7bda34c; rom[381] = 32'ha72ca3d7; rom[382] = 32'ha69ca463; rom[383] = 32'ha60ca4f0;
        rom[384] = 32'ha57ea57e; rom[385] = 32'ha4f0a60c; rom[386] = 32'ha463a69c; rom[387] = 32'ha3d7a72c;
        rom[388] = 32'ha34ca7bd; rom[389] = 32'ha2c2a84f; rom[390] = 32'ha238a8e2; rom[391] = 32'ha1b0a976;
        rom[392] = 32'ha129aa0a; rom[393] = 32'ha0a2aaa0; rom[394] = 32'ha01cab36; rom[395] = 32'h9f98abcd;
        rom[396] = 32'h9f14ac65; rom[397] = 32'h9e91acfd; rom[398] = 32'h9e0fad97; rom[399] = 32'h9d8eae31;
        rom[400] = 32'h9d0eaecc; rom[401] = 32'h9c8faf68; rom[402] = 32'h9c11b005; rom[403] = 32'h9b94b0a2;
        rom[404] = 32'h9b17b140; rom[405] = 32'h9a9cb1df; rom[406] = 32'h9a22b27f; rom[407] = 32'h99a9b31f;
        rom[408] = 32'h9930b3c0; rom[409] = 32'h98b9b462; rom[410] = 32'h9843b505; rom[411] = 32'h97ceb5a8;
        rom[412] = 32'h9759b64c; rom[413] = 32'h96e6b6f1; rom[414] = 32'h9674b796; rom[415] = 32'h9603b83c;
        rom[416] = 32'h9592b8e3; rom[417] = 32'h9523b98b; rom[418] = 32'h94b5ba33; rom[419] = 32'h9448badc;
        rom[420] = 32'h93dcbb85; rom[421] = 32'h9371bc2f; rom[422] = 32'h9307bcda; rom[423] = 32'h929ebd86;
        rom[424] = 32'h9236be32; rom[425] = 32'h91cfbedf; rom[426] = 32'h9169bf8c; rom[427] = 32'h9105c03a;
        rom[428] = 32'h90a1c0e9; rom[429] = 32'h903ec198; rom[430] = 32'h8fddc248; rom[431] = 32'h8f7dc2f8;
        rom[432] = 32'h8f1dc3a9; rom[433] = 32'h8ebfc45b; rom[434] = 32'h8e62c50d; rom[435] = 32'h8e06c5c0;
        rom[436] = 32'h8dabc673; rom[437] = 32'h8d51c727; rom[438] = 32'h8cf8c7db; rom[439] = 32'h8ca1c890;
        rom[440] = 32'h8c4ac946; rom[441] = 32'h8bf5c9fc; rom[442] = 32'h8ba0cab2; rom[443] = 32'h8b4dcb69;
        rom[444] = 32'h8afbcc21; rom[445] = 32'h8aaaccd9; rom[446] = 32'h8a5acd92; rom[447] = 32'h8a0cce4b;
        rom[448] = 32'h89becf04; rom[449] = 32'h8972cfbe; rom[450] = 32'h8927d079; rom[451] = 32'h88ddd134;
        rom[452] = 32'h8894d1ef; rom[453] = 32'h884cd2ab; rom[454] = 32'h8805d367; rom[455] = 32'h87c0d424;
        rom[456] = 32'h877bd4e1; rom[457] = 32'h8738d59e; rom[458] = 32'h86f6d65c; rom[459] = 32'h86b6d71b;
        rom[460] = 32'h8676d7d9; rom[461] = 32'h8637d898; rom[462] = 32'h85fad958; rom[463] = 32'h85beda18;
        rom[464] = 32'h8583dad8; rom[465] = 32'h8549db99; rom[466] = 32'h8511dc59; rom[467] = 32'h84d9dd1b;
        rom[468] = 32'h84a3dddc; rom[469] = 32'h846ede9e; rom[470] = 32'h843adf61; rom[471] = 32'h8407e023;
        rom[472] = 32'h83d6e0e6; rom[473] = 32'h83a6e1a9; rom[474] = 32'h8377e26d; rom[475] = 32'h8349e330;
        rom[476] = 32'h831ce3f4; rom[477] = 32'h82f1e4b9; rom[478] = 32'h82c6e57d; rom[479] = 32'h829de642;
        rom[480] = 32'h8276e707; rom[481] = 32'h824fe7cd; rom[482] = 32'h822ae892; rom[483] = 32'h8205e958;
        rom[484] = 32'h81e2ea1e; rom[485] = 32'h81c1eae4; rom[486] = 32'h81a0ebab; rom[487] = 32'h8181ec71;
        rom[488] = 32'h8163ed38; rom[489] = 32'h8146edff; rom[490] = 32'h812aeec6; rom[491] = 32'h8110ef8d;
        rom[492] = 32'h80f6f055; rom[493] = 32'h80def11c; rom[494] = 32'h80c8f1e4; rom[495] = 32'h80b2f2ac;
        rom[496] = 32'h809ef374; rom[497] = 32'h808bf43c; rom[498] = 32'h8079f505; rom[499] = 32'h8068f5cd;
        rom[500] = 32'h8059f695; rom[501] = 32'h804bf75e; rom[502] = 32'h803ef827; rom[503] = 32'h8032f8ef;
        rom[504] = 32'h8027f9b8; rom[505] = 32'h801efa81; rom[506] = 32'h8016fb4a; rom[507] = 32'h800ffc13;
        rom[508] = 32'h800afcdc; rom[509] = 32'h8006fda5; rom[510] = 32'h8002fe6e; rom[511] = 32'h8001ff37;
    end

    assign w_real = $signed(rom[addr][31:16]);
    assign w_imag = $signed(rom[addr][15:0]);

endmodule
