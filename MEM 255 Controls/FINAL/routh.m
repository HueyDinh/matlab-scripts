syms gam;
arr_1 = [1 (8420000+2000*gam)/72000 140000/72000*gam 0 0];
arr_2 = [16840/72000 280*gam/72000 0 0 0];
arr_3 = -1/arr_2(1)*[det([arr_1(1) arr_1(2);arr_2(1) arr_2(2)]) det([arr_1(1) arr_1(3);arr_2(1) arr_2(3)]) det([arr_1(1) arr_1(4);arr_2(1) arr_2(4)])];
arr_4 = -1/arr_3(1)*[det([arr_2(1) arr_2(2);arr_3(1) arr_3(2)]) det([arr_2(1) arr_2(3);arr_3(1) arr_3(3)])];
arr_5 = -1/arr_4(1)*[det([arr_3(1) arr_3(2);arr_4(1) arr_4(2)])];
pretty(arr_3)
pretty(simplify(arr_4))
pretty(simplify(arr_5))