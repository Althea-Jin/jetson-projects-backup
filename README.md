# Jetson projects backup

Backup prepared on nvidia-desktop, 2026-11-02.

## Contents

Nine project snapshots: LightNav_Project, fastlio_diag, JEP,
jetson-inference, lio_ws, Livox-SDK2, livox_ws, scan_ws, ultralytics.
Also includes Documents, Desktop, RViz configuration and system/package information.

## Scope and restore notes

This is a filtered source/configuration snapshot, not a complete disk image.
Original Git histories and submodule Git pointers are omitted; available submodule
source files are included as ordinary files. Named build/install/log/cache and
virtual-environment directories, common recording formats and files over 95 MiB
were excluded during copying. Existing binaries and model weights below that
threshold may be included. Dependencies and excluded data may need reinstalling
or restoring separately.

The OPENAI_API_KEY assignment was removed from the backed-up shell configuration.
Authentication fields in backed-up NoMachine connection files were cleared.
Original projects and original environment configuration were not modified.
Do not copy unreviewed credentials, private keys or raw shell configuration here.
