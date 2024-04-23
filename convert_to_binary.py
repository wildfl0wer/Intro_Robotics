def convert_to_binary(decimal_number):
    """
    Converts a decimal number to a binary string
    
    This is done by assigning the value of
    the variable decimal_number mod 2 to the
    variable remainder, appending remainder to the
    list binary_digits,and assigning the value of
    decimal_number divided 2 to decimal_number.
    These steps are repeated while decimal_number
    is greater than 0.
    After the calculation of the binary number is
    finished, the numbers in binary_digits are
    joined to an empty string in reverse order.
    After this, the "0b" is added to the binary
    number to create a binary string.
    
    Parameters:
        decimal_number: Decimal number to convert
    Returns:
        Binary string
    """
    
    if decimal_number == 0:
        return "0b0"           # Special case: decimal 0

    binary_digits = []         # Stores binary digits

    # Calculates the binary equivalent of decimal_number
    while decimal_number > 0:
        remainder = decimal_number % 2
        binary_digits.append(str(remainder))
        decimal_number //= 2

    # Reverses binary_digits to correctly represent binary number
    binary_string = "".join(reversed(binary_digits))

    # Adds prefix "0b" to indicate Python binary string
    binary_string = "0b" + binary_string
    
    return binary_string

print(convert_to_binary(10))
