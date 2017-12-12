caase = 4;

if caase == 1
runlocalization_MCL('so_o3_ie.txt', 'map_o3.txt', 1, 1, 1,[], 2)
elseif caase == 2
runlocalization_MCL('so_pb_10_outlier.txt', 'map_pent_big_10.txt', 1, 1, 1,[0; 0; 0], 2)
elseif caase == 3
runlocalization_MCL('so_pb_40_no.txt', 'map_pent_big_40.txt', 1, 1, 1,[], 2)
elseif caase == 4
runlocalization_MCL('so_sym2_nk.txt', 'map_sym2.txt', 1, 1, 1,[], 2)
else
runlocalization_MCL('so_sym3_nk.txt', 'map_sym3.txt', 1, 1, 1,[], 2)    
end