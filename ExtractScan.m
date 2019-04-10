function data = ExtractScan(scan)
    MaskLow13Bits = uint16(2^13-1); % mask for extracting the range bits.
    maskE000 = bitshift(uint16(7),13)  ;
    data.intensity = bitand(scan,maskE000);
    rangesA = bitand(scan,MaskLow13Bits) ;
    data.ranges = 0.01*double(rangesA);
return;
end