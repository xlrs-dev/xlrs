#!/bin/bash
# generic picotool wrapper to avoid zsh hook issues
picotool "$@" 2>&1
echo "EXIT:$?"
