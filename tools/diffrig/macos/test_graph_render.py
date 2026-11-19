#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Parser tests for graph_render.py on a synthetic graph (no tuning data)."""
import importlib.util
import os
import unittest

spec = importlib.util.spec_from_file_location("gr", os.path.join(os.path.dirname(__file__), "graph_render.py"))
gr = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(gr)
except ImportError:  # numpy missing
    gr = None

GRAPH = """graphName "t"
[def numChansIn 2]
in in
out out
box NFP nfnp 1 1 ; comment
box split split 1 2
box sum sum 2 1
box Gain dbgn 1 1
box A (aufx tmst) 1 1
box P (aufx atsp) 1 1
wire in NFP ([fs] [numChansIn])
wire NFP split ([fs] [numChansIn])
wire (split 0) (sum 0) ([fs] 1)
wire (split 1) (sum 1) ([fs] 1)
wire sum Gain ([fs] 1)
wire Gain A ([fs] 1)
wire A P ([fs] 1)
wire P out ([fs] 1)
param mvol 0
param mvsc 0
wireGraphParam mvol (Gain 0)
wireGraphParam mvsc (A 3 0 0)
"""


@unittest.skipIf(gr is None, "numpy not installed")
class Parse(unittest.TestCase):
    def test_graph(self):
        defs, boxes, wires, params, gparam = gr.parse_dspg(GRAPH)
        self.assertEqual(defs["numChansIn"], "2")
        self.assertEqual(boxes["A"]["type"], "tmst")
        self.assertTrue(boxes["A"]["au"])
        self.assertEqual(boxes["sum"]["nin"], 2)
        self.assertEqual(wires[0]["ch"], 2)
        self.assertEqual(wires[2]["src"], ("split", 0))
        self.assertEqual(wires[3]["dst"], ("sum", 1))
        self.assertIn(("mvsc", "A", 3), gparam)

    def test_non_global_param_refused(self):
        with self.assertRaises(SystemExit):
            gr.parse_dspg(GRAPH + "wireGraphParam mvsc (A 3 1 0)\n")


if __name__ == "__main__":
    unittest.main()
