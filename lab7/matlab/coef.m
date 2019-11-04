% hpf and lpf scripts are expected to have run before executing this script
hpcoef = hpfilter.Numerator;
lpcoef = lpfilter.Numerator;

% create the C useable coefficient table
hpfCoefPrintout = sprintf('\n%0.18ff, %0.18ff, %0.18ff, %0.18ff, ', hpcoef);
lpfCoefPrintout = sprintf('\n%0.18ff, %0.18ff, %0.18ff, %0.18ff, ', lpcoef);

disp("HIGH PASS COEFFICIENTS")
hpfCoefPrintout(1:end-1)

disp("LOW PASS COEFFICIENTS")
lpfCoefPrintout(1:end-1)
