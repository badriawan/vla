# Franka MuJoCo + π0.5

Workspace kecil untuk mengetes loop inferensi π0.5 pada robot Franka di MuJoCo.
Scene berisi lengan 7-DoF, gripper paralel, meja, kubus merah, kamera eksternal,
dan kamera wrist.

## Mulai cepat

Python 3.10+ disarankan.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
MUJOCO_GL=egl franka-pi05 --mode mock --steps 30 --save-frames rollouts/mock
pytest
```

Pada macOS, hapus `MUJOCO_GL=egl`. Mode `mock` tidak memerlukan GPU atau model dan
digunakan untuk memeriksa scene, kamera, dimensi observasi, serta kontrol robot.
Rendering offscreen pada macOS perlu dijalankan dari sesi desktop (CoreGraphics);
terminal CI tanpa display tetap dapat menjalankan test fisika yang disediakan.
Untuk smoke test CLI pada terminal tersebut gunakan `--no-render` dan jangan kirim
hasilnya ke model sebagai evaluasi visual.

## Menjalankan π0.5

Gunakan konfigurasi/checkpoint DROID pada server OpenPI karena adapter ini mengikuti
kontrak DROID: aksi adalah 7 joint velocity + 1 gripper position.

Di checkout OpenPI (mesin GPU):

```bash
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_droid \
  --policy.dir=/path/ke/checkpoint-pi05-droid
```

Pasang client dari checkout OpenPI ke environment workspace ini:

```bash
pip install -e /path/ke/openpi/packages/openpi-client
```

Kemudian jalankan rollout:

```bash
franka-pi05 --mode remote \
  --host localhost --port 8000 \
  --prompt "pick up the red cube" \
  --steps 300 --open-loop-horizon 8 \
  --save-frames rollouts/pi05
```

Server mengembalikan action chunk `[horizon, 8]`. Client meminta chunk baru setiap
8 langkah (sekitar 0,53 detik pada 15 Hz), sesuai pola evaluasi DROID OpenPI.

## Kontrak observasi

- `observation/exterior_image_1_left`: RGB `uint8`, `[224, 224, 3]`
- `observation/wrist_image_left`: RGB `uint8`, `[224, 224, 3]`
- `observation/joint_position`: `float32`, `[7]`
- `observation/gripper_position`: `float32`, `[1]`, 0 tertutup dan 1 terbuka
- `prompt`: instruksi bahasa

Model robot di sini merupakan MJCF ringan untuk integrasi dan smoke test. Untuk
evaluasi dinamika yang lebih mendekati hardware, ganti scene dengan model resmi
`franka_emika_panda` dari MuJoCo Menagerie sambil mempertahankan nama joint,
kamera, dan kontrak I/O di `FrankaMuJoCoEnv`.
