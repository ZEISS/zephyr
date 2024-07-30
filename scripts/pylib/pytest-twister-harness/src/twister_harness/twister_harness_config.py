# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import logging
from dataclasses import dataclass, field
from pathlib import Path
from twister_harness.helpers.domains_helper import get_default_domain_name

import pytest

logger = logging.getLogger(__name__)


@dataclass
class DeviceConfig:
    type: str
    build_dir: Path
    base_timeout: float = 60.0  # [s]
    platform: str = ''
    properties: list[str] = field(default_factory=list, repr=False)
    serial: str = ''
    baud: int = 115200
    runner: str = ''
    runner_params: list[str] = field(default_factory=list, repr=False)
    id: str = ''
    product: str = ''
    serial_pty: str = ''
    flash_before: bool = False
    west_flash_extra_args: list[str] = field(default_factory=list, repr=False)
    name: str = ''
    pre_script: Path | None = None
    post_script: Path | None = None
    post_flash_script: Path | None = None
    fixtures: list[str] = None
    app_build_dir: Path | None = None

    def __post_init__(self):
        domains = self.build_dir / 'domains.yaml'
        if domains.exists():
            self.app_build_dir = self.build_dir / get_default_domain_name(domains)
        else:
            self.app_build_dir = self.build_dir


@dataclass
class TwisterHarnessConfig:
    """Store Twister harness configuration to have easy access in test."""
    devices: list[DeviceConfig] = field(default_factory=list, repr=False)

    @classmethod
    def create(cls, options: argparse.Namespace) -> TwisterHarnessConfig:
        """Create new instance from pytest.Config."""

        devices = []

        west_flash_extra_args: list[str] = []
        if options.west_flash_extra_args:
            west_flash_extra_args = [w.strip() for w in options.west_flash_extra_args.split(',')]
        device_from_cli = DeviceConfig(
            type=options.device_type,
            build_dir=_cast_to_path(options.build_dir),
            base_timeout=options.base_timeout,
            platform=options.platform,
            properties=options.device_properties,
            serial=options.device_serial,
            baud=options.device_serial_baud,
            runner=options.runner,
            id=options.device_id,
            product=options.device_product,
            serial_pty=options.device_serial_pty,
            west_flash_extra_args=west_flash_extra_args,
            pre_script=_cast_to_path(options.pre_script),
            post_script=_cast_to_path(options.post_script),
            post_flash_script=_cast_to_path(options.post_flash_script),
        )

        devices.append(device_from_cli)

        return cls(
            devices=devices
        )


def _cast_to_path(path: str | None) -> Path | None:
    if path is None:
        return None
    return Path(path)
