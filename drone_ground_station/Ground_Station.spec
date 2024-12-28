# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(
    ['Ground_Station.py'],
    pathex=[],
    binaries=[],
    datas=[('resources/config.csv', 'resources'), ('resources/log_path.txt', 'resources')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='Ground_Station',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['khamsin.icns'],
)
app = BUNDLE(
    exe,
    name='Ground_Station.app',
    icon='khamsin.icns',
    bundle_identifier=None,
)
