#!/usr/bin/env python3
"""Compatibility entry point; launch files use operator_web_node.py."""
from peaceofmine_operator.web_bridge import OperatorWebBridge as OperatorGateway

def main():
    from operator_web_node import main as serve
    serve()

if __name__ == '__main__':
    main()
