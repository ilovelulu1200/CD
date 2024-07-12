function fixed_point_conversion()
    % Define the number format
    int_bits = 6; % number of integer bits
    frac_bits = 10; % number of fractional bits

    % Example inputs
    a = 0.044921875; % example input in decimal
    b = 0.7978515625; % example input in decimal

    % Convert to fixed point representation
    a_fixed = int_to_fixed_point(a, int_bits, frac_bits);
    b_fixed = int_to_fixed_point(b, int_bits, frac_bits);

    % Display results
    fprintf('Fixed Point Representation of %d:\n', a);
    fprintf('Binary: %s\n', dec2bin(a_fixed, int_bits + frac_bits));
    fprintf('Decimal: %f\n\n', from_fixed_point(a_fixed, int_bits, frac_bits));

    fprintf('Fixed Point Representation of %d:\n', b);
    fprintf('Binary: %s\n', dec2bin(b_fixed, int_bits + frac_bits));
    fprintf('Decimal: %f\n', from_fixed_point(b_fixed, int_bits, frac_bits));

    % Function to convert an integer to fixed point
    function fixed = int_to_fixed_point(x, int_bits, frac_bits)
        % Scale the integer to fixed point
        fixed = round(x * 2^frac_bits);
        % Handle negative values for signed representation
        if x < 0
            fixed = bitset(abs(fixed), int_bits + frac_bits);
        end
    end

    % Function to convert fixed point to decimal
    function dec = from_fixed_point(x, int_bits, frac_bits)
        %A scaling factor used to convert a fractional part to an integer part for representation and processing in fixed-point format.
        scale = 2^frac_bits;
        % Handle negative values for signed representation
        if bitget(x, int_bits + frac_bits)
            x = -bitset(abs(x), int_bits + frac_bits, 0);
        end
        dec = x / scale;
    end
end