# Elarion hardware build settings.
#
# The Makefile includes this file before it defaults CONFIG_DIR, so pointing
# CONFIG_DIR at the in-tree elarion/ directory here means the ELARION targets
# build straight out of a plain clone:
#
#     make ELARIONH743
#
# No CONFIG_DIR= flag, no BETAFLIGHT_CONFIG environment variable and no
# src/config submodule checkout are needed. Upstream ships this file gitignored
# as a developer scratch file; we track it deliberately, so the corresponding
# line has been removed from .gitignore.
CONFIG_DIR := $(ROOT)/elarion
