"""The evaluation harness.

``parkscan_eval`` measures the *perception pipeline* against known simulated ground truth
and writes a reproducible ``results.csv`` / ``results.json`` pair. Read ``EVAL.md`` next to
this package before quoting any number it produces: every figure it can generate is Tier 1
(pipeline behaviour in simulation), and the document says explicitly what Tier 2 and Tier 3
figures would require and why they are absent.
"""
