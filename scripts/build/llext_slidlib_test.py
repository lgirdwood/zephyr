#!/usr/bin/env python3
# Copyright 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""
Tests for llext_slidlib.py
"""

import unittest
from unittest import mock
import sys
import os

# Add scripts/build to python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import llext_slidlib


class TestLlextSlidLib(unittest.TestCase):
    """Tests for LLEXT SLID generation and signature hashing library."""

    def test_slid_generation_fallback(self):
        # With signature generation mocked to return None (name-only fallback)
        with mock.patch("llext_slidlib.find_signature", return_value=None):
            # Known direct sha256 fallback SLIDs
            self.assertEqual(llext_slidlib.generate_slid("rzalloc", 4), 0x2044F1BD)
            self.assertEqual(llext_slidlib.generate_slid("rfree", 4), 0x42435544)

    @mock.patch("builtins.open")
    @mock.patch("subprocess.run")
    @mock.patch("os.path.isdir", return_value=True)
    def test_find_signature_parsing(self, mock_isdir, mock_run, mock_open):
        # Setup mock for ripgrep search results
        mock_res = mock.Mock()
        mock_res.stdout = "/workspace/sof/include/audio.h:2:int my_func(int a, char *b);\n"
        mock_run.return_value = mock_res

        # Setup mock file lines returned by readlines()
        file_content = [
            "/* header info */\n",
            "int my_func(int a, char *b);\n"
        ]
        mock_open.return_value.__enter__.return_value.readlines.return_value = file_content

        # Execute signature extraction
        llext_slidlib._signature_cache.clear() # Clear cache to force search
        sig = llext_slidlib.find_signature("my_func")
        
        # Verify normalizations
        self.assertEqual(sig, "int my_func(int,char*)")

    @mock.patch("builtins.open")
    @mock.patch("subprocess.run")
    @mock.patch("os.path.isdir", return_value=True)
    def test_find_signature_void_params(self, mock_isdir, mock_run, mock_open):
        mock_res = mock.Mock()
        mock_res.stdout = "/workspace/sof/include/audio.h:1:void init_device(void);\n"
        mock_run.return_value = mock_res

        file_content = ["void init_device(void);\n"]
        mock_open.return_value.__enter__.return_value.readlines.return_value = file_content

        llext_slidlib._signature_cache.clear()
        sig = llext_slidlib.find_signature("init_device")
        self.assertEqual(sig, "void init_device(void)")

    @mock.patch("builtins.open")
    @mock.patch("subprocess.run")
    @mock.patch("os.path.isdir", return_value=True)
    def test_find_signature_struct_ptrs(self, mock_isdir, mock_run, mock_open):
        mock_res = mock.Mock()
        mock_res.stdout = "/workspace/sof/include/audio.h:1:struct comp_dev *comp_new(const struct comp_ipc_config *config);\n"
        mock_run.return_value = mock_res

        file_content = ["struct comp_dev *comp_new(const struct comp_ipc_config *config);\n"]
        mock_open.return_value.__enter__.return_value.readlines.return_value = file_content

        llext_slidlib._signature_cache.clear()
        sig = llext_slidlib.find_signature("comp_new")
        self.assertEqual(sig, "struct comp_dev *comp_new(const struct comp_ipc_config*)")


if __name__ == "__main__":
    unittest.main()
