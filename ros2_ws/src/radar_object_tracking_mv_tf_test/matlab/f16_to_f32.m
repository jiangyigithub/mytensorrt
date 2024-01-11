function f32Mat = f16_to_f32(f16Mat)
% Slice the sixteen bits into respective parts of half-precision float
f16Vec      = f16Mat(:);
signBit_16  = (f16Vec - mod(f16Vec, 2^15)) / 2^15;
exponent_16 = mod((f16Vec - mod(f16Vec, 2^10)) / 2^10, 2^5);
mantissa_16 = mod(f16Vec, 2^10);

% Translate from half-precision bits to single-recision bits
signBit_32  = signBit_16;
exponent_32 = exponent_16 + 112;
mantissa_32 = mantissa_16 * 2^13;

% Concatenate the respective parts and cast to float
f32Vec = typecast(uint32(signBit_32 * 2^31 + exponent_32 * 2^23 + mantissa_32), 'single');

% Deal with special cases
f32Vec(exponent_16 == 31)                   = NaN;
f32Vec(mantissa_16 == 0 & exponent_16 == 0) = 0;
f32Mat                                      = double(reshape(f32Vec, size(f16Mat)));
end
