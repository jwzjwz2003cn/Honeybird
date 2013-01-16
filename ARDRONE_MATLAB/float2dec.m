function dec_num = float2dec( float_num )
%FLOAT2DEC Summary of this function goes here
%   Detailed explanation goes here

    single_float = single(float_num);
    signed_hex = num2hex(single_float);
    dec_num = typecast(uint32(hex2dec(signed_hex)),'int32');
   
end

