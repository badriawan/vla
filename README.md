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

Preview resolusi lebih tinggi di macOS:

```bash
franka-pi05 --mode mock --steps 30 --image-size 640 --save-frames rollouts/panda
```

## Teleop keyboard (macOS)

```bash
source .venv/bin/activate
python -m franka_pi05.teleop
```

Klik jendela teleop untuk memberi fokus. Tekan **1–7** untuk memilih joint,
tahan **panah atas/bawah** untuk menggerakkan joint tersebut. Lepaskan untuk
menghentikan perintah gerak. **G/H** membuka/menutup gripper, **+/−** mengatur
kecepatan, **Space** pause/resume, **R** reset pose, **Esc** keluar.
Scroll untuk zoom. Saat fokus jendela hilang, simulasi otomatis pause;
tekan Space setelah kembali. Ini kontrol joint di simulasi, bukan kontrol TCP
Cartesian atau koneksi ke robot fisik.

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

Model robot memakai mesh, kinematika, dan inertial Franka Emika Panda dari
MuJoCo Menagerie (Apache 2.0; sumber dan lisensi ada di `assets/panda`). Scene
menambahkan meja, kubus, kamera eksternal dan wrist, serta kontrol velocity
untuk kontrak I/O workspace. Ini tetap environment integrasi/smoke test;
kontrol dan domain kamera belum dikalibrasi terhadap hardware DROID.
