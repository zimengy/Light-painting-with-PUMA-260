function vals = linear_trajectory_kuchenbe(t, tvia1, tvia2, valsvia1, valsvia2)

vals = valsvia1 + (t - tvia1) * (valsvia2 - valsvia1) / (tvia2 - tvia1);
