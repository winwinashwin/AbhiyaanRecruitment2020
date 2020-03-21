"""
This is an Arithmetic expression evaluator using stacks in python.
I have made use of the deque class in collections module to implement the concept of stack.
So far, I have only included addition, subtraction, multiplication and division over the range of decimal numbers
"""

# import necessary modules
from collections import deque
import re


# function to validate expression
def validate(expression):
    try:
        eval(expression)
    except Exception as e:
        print("ERROR : Expression is invalid.")
        print(e)
        quit(1)

    # remove white spaces and replace certain combinations
    expression = expression.replace(" ", "")
    expression = expression.replace("+-", "-").replace("-+", "-").replace("++", "+").replace("--", "+")

    # substitute - with +- so as to facilitate negative integers in the expression
    expression = re.sub(r'(?<=\d)(-)(?=\d)', "+-", expression)
    expression = re.sub(r'(?<=\d)(\+)(?=\d)', "++", expression)

    if '**' in expression or '//' in expression:
        print("ERROR : Invalid operators found in expression.")
        quit(1)

    print('>>> Expression validated successfully.')
    # returns a list of characters in the expressions
    return list(filter(lambda a: a != "", re.split(r'((?:-\d+)|(?:\+\d+)|(?:\d+)|(?:[-+*/()]))', expression)))


# function to evaluate an expression
def evaluate(operator, operand2, operand1):
    return str(eval(operand1 + operator + operand2))


# check precedence of operators
def has_precedence(operator1, operator2):
    global OPERATORS

    if operator2 in OPERATORS[4:]:
        return False
    elif (operator1 == '*' or operator1 == '/') and (operator2 == '+' or operator2 == '-'):
        return False
    else:
        return True


class Stack(deque):
    def __init__(self):
        super().__init__()

    # push data to stack
    def push(self, *args, **kwargs):
        self.append(*args, **kwargs)

    # Returns whether the stack is empty
    def is_empty(self):
        return not bool(self)

    # Returns a reference to the top most element of the stack
    def top(self):
        if not self.is_empty():
            return self[-1]
        else:
            return None

    # Returns the size of the stack
    def size(self):
        return len(self)

    # pop function is already built into deque class
    pass


if __name__ == "__main__":
    OPERATORS = ['+', '-', '*', '/', '(', ')']

    expr = input("\n>>> Enter an arithmetic expression : ")

    # stacks for operators and operands in the expression
    operands = Stack()
    operators = Stack()

    elements = validate(expr)
    # print(elements)

    for element in elements:
        if element not in OPERATORS:
            operands.push(element)
        elif element == '(':
            operators.push(element)
        elif element == ')':
            while operators.top() != '(':
                operands.push(evaluate(operators.pop(), operands.pop(), operands.pop()))
            operators.pop()
        elif element in OPERATORS[:4]:
            while not operators.is_empty() and has_precedence(element, operators.top()):
                operands.push(evaluate(operators.pop(), operands.pop(), operands.pop()))
            operators.push(element)
        else:
            pass

    # the expression has been completely parsed by this point
    while not operators.is_empty():
        operands.push(evaluate(operators.pop(), operands.pop(), operands.pop()))

    print(f"\n\tResult : {operands.pop()}")
    quit(0)
