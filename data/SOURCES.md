# Test data sources (heart_cam)

Downloaded 2026-09-17. Total on disk: about 345 MB. Every video was checked with ffprobe and opens in OpenCV (`.venv`).

---

## 1. Face videos with ground truth (rPPG)

### `ubfc_rppg_dataset2_subject1/` (UBFC-rPPG, DATASET_2, subject1)
| File | Details |
|---|---|
| `vid_first27s_ffv1.mkv` | 640x480, 29.26 fps (container value), 802 frames, 27.4 s, **FFV1 lossless** |
| `ground_truth.txt` | contact PPG from a CMS50E pulse oximeter, covering the **full 52.7 s** recording |
| `ubfcrppg_data_processor.py`, `readme_ubfc.txt` | official reader script and readme |

- **Source:** https://sites.google.com/view/ybenezeth/ubfcrppg. The Google Drive folder is https://drive.google.com/drive/folders/1o0XU4gTIo46YfwaWjIgbtCncc-oF44Xk and the original file id is `1qBlkbaB8y3-KlWC_A61KsY42Wjo66ss4`.
- **How the clip was made:** the original `vid.avi` is uncompressed BGR24 and 1.3 GB. Only its first 740 MB was streamed (HTTP range request) and re-encoded losslessly to FFV1, so the pixel values are identical to the source for frames 0–801.
- **Terms:** shared "for research purpose". You must cite Bobbia et al., "Unsupervised skin tissue segmentation for remote photoplethysmography", Pattern Recognition Letters 2017. The Drive folder also holds an `Agreement.xlsx`, which was not signed. The readme says the faces of subjects 21 and 27 must not be published (this is subject 1).
- **Ground-truth format** (`ground_truth.txt`, 3 lines, whitespace-separated, scientific notation, 1547 samples each):
  - line 1: PPG waveform (normalised, arbitrary units)
  - line 2: HR in bpm, as reported by the oximeter
  - line 3: timestamp in seconds, starting at 0. The effective rate is about 29.36 Hz, roughly one sample per video frame.
- **Alignment:** use the timestamps. The data is synchronised with the frames. Only samples with t ≤ 27.4 s fall inside this clip.
- **Expected HR:** 97–110 bpm (median **102 bpm**) over 0–27.4 s. The median over the full recording is 108 bpm. The authors recommend computing reference HR from the PPG waveform, not from line 2.

### `mpu_rppg_sample/` (MPU-rPPG sample subset)
| File | Details |
|---|---|
| `Output.mp4` | 640x480, 60 fps, 34 496 frames, 574.9 s (9.6 min), MPEG-4 Part 2 (lossy). Seated subject using a computer, face visible, some movement. |
| `Output.csv` | ground truth, 34 496 rows (one per frame) |

- **Source:** Figshare "MPU-rPPG Sample Dataset", DOI 10.6084/m9.figshare.29377835. The files came from https://ndownloader.figshare.com/files/55553429 (video) and /55553426 (csv). This is one of five video/csv pairs; the others are 128–175 MB each, plus one 7.2 GB mkv. Code is at https://github.com/kingflyingsnow/MPU-RPPG-WORK (MIT).
- **Terms:** Figshare lists **CC BY 4.0**. The paper (Scientific Data 2026) states CC BY-NC-ND 4.0, so treat it as non-commercial.
- **Ground-truth format:** CSV with header `Count,PPG,HR,SPO2`.
  - `Count`: frame index at 60 Hz
  - `PPG`: raw pulse-oximeter waveform, integer 0–127 (CMS60D). The first few rows are 0.
  - `HR`: bpm
  - `SPO2`: %
- **Expected HR:** 67–83 bpm, median 71, mean about 73 bpm.

---

## 2. MIT Eulerian Video Magnification source videos (`evm/`)
- **Source:** https://people.csail.mit.edu/mrub/evm/ (files under `video/`).
- **Paper:** Wu et al., "Eulerian Video Magnification for Revealing Subtle Changes in the World", SIGGRAPH 2012.
- **Terms:** provided for research use. Cite the paper. No explicit license.
- **Ground truth:** none, only the known pulse or breathing and the paper's filter bands.

| File | Res | fps | Dur | Notes |
|---|---|---|---|---|
| `face.mp4` | 528x592 | 30 | 10.0 s | Face pulse. The paper magnifies 0.83–1.0 Hz, so expect about **50–60 bpm**. |
| `face2.mp4` | 570x718 | 30 | 10.0 s | Face pulse, same band (0.83–1.0 Hz), so expect about 50–60 bpm. |
| `baby.mp4` | 960x544 | 30 | 10.0 s | Sleeping baby. Breathing motion (the paper uses motion magnification with an IIR filter). |
| `baby2.mp4` | 640x352 | 30 (29.9999) | 30.0 s | Newborn with a hospital monitor in view. The paper magnifies 2.33–2.67 Hz, so expect **about 140–160 bpm**. |
| `wrist.mp4` | 640x352 | 30 | 29.8 s | Wrist/radial-artery pulse (motion). |
| `face-ideal-from-0.83333-to-1-alpha-50-level-4-chromAtn-1.mp4` | 528x592 | 30 | 9.7 s | MIT's magnified result for `face.mp4`, used as a reference output. |
| `baby-iir-r1-0.4-r2-0.05-alpha-10-lambda_c-16-chromAtn-0.1.mp4` | 960x544 | 30 | 9.7 s | MIT's magnified result for `baby.mp4`. |

---

## 3. Infant breathing with annotations (`air400_infant_breathing/`)
- **Source:** AIR-400 / AIR-125, Northeastern ACLab.
  - Download site: https://coe.northeastern.edu/Research/AClab/AIR-400/ (open Apache directory listing)
  - Code: https://github.com/michaelwwan/air-400 (MIT license)
  - Paper: Song et al., WACV 2026
  - The original AIR-125 is at https://coe.northeastern.edu/Research/AClab/AIR-125/ (`AIR.zip`, 947 MB, not downloaded).
- **Terms:** the repo is MIT licensed, and the data is public with no form. The footage comes from real infant baby monitors, so do not redistribute it.

| File | Details |
|---|---|
| `S01_1.mp4`, `S01_2.mp4`, `S01_3.mp4` | 1920x1080, 10 fps, about 60 s each. Night-vision (IR, greyscale) crib camera, H.264 with an AAC audio track. From `AIR-400/S01/{1,2,3}.mp4`. |
| `S05_1.mp4` | same format, subject S05 (`AIR-400/S05/1.mp4`) |
| `demo-air-400-s05-23.mp4` | 1920x1080, 10 fps, 60.8 s. The authors' demo clip; no annotation was downloaded. |
| `S0x_n.hdf5` | Annotations, from `AIR-400/S0x/out/n.hdf5`. |

- **Annotation format (HDF5):**
  - `filename`: bytes, the name of the source video
  - `respiration`: float64 array of shape (600,), **one value per video frame at 10 Hz**, covering the first 60 s
  - `respiration` is a train of Gaussian-smoothed impulses, one per annotated breath, and each impulse sums to 1. So `respiration.sum()` equals the number of breaths in 60 s, which is the rate in breaths/min.
  - Reading it requires `h5py`, which is not installed in `.venv`.
- **Expected respiration rate:** S01_1 = **18**, S01_2 = **19**, S01_3 = **19**, S05_1 = **27** breaths/min.
- Each subject folder on the server also has `N_waveform.png` and `N_impulse.png` plots, a VIA annotation JSON, and an `original_videos/` directory.

---

## Datasets found that need registration or an EULA (not downloaded)
| Dataset | URL | Access |
|---|---|---|
| PURE (TU Ilmenau) | https://www.tu-ilmenau.de (Neuroinformatics group: "PURE pulse rate detection dataset") | request by email |
| COHFACE (Idiap) | https://www.idiap.ch/en/scientific-research/data/cohface | Idiap account and EULA |
| MMPD | https://github.com/McJackTang/MMPD_rPPG_dataset | signed EULA form |
| MAHNOB-HCI | https://mahnob-db.eu/hci-tagging/ | registration and EULA |
| VIPL-HR | ICT/CAS VIPL lab | release agreement form |
| UBFC-Phys | https://ieee-dataport.org/open-access/ubfc-phys-2 | IEEE DataPort login |
| MPSC-rPPG | https://ieee-dataport.org/documents/mpsc-rppg-dataset | IEEE DataPort login |
| VitalVideos (Europe and others) | https://vitalvideos.org/ | request form from an academic email |
| LGI-PPGI-DB | https://github.com/partofthestars/LGI-PPGI-DB | the video downloads are offline (hosting withdrawn) |
| UBFC-rPPG on Kaggle | https://www.kaggle.com/datasets/malekdinarito/ubfc-rppg-dataset | Kaggle login. The official Google Drive link above works without login. |
| rPPG-Toolbox / pyVHR | https://github.com/ubicomplab/rPPG-Toolbox | no sample video included; it relies on the datasets above |

## Possible extra downloads
- **Other UBFC-rPPG subjects:** 1.3–2.1 GB raw each, but they can be range-streamed the same way. Example:
  `curl -sL -r 0-740000000 "https://drive.usercontent.google.com/download?id=<ID>&export=download&confirm=t" | ffmpeg -f avi -i pipe:0 -c:v ffv1 out.mkv`
  - Folder ids can be listed with `https://drive.google.com/embeddedfolderview?id=<folder id>`.
  - DATASET_1 (`gtdump.xmp`, CSV with columns time in ms, HR, SpO2, PPG) has the `after-exercise` session, which gives a higher HR.
- **More MPU-rPPG pairs:** the other files listed in the Figshare API response at https://api.figshare.com/v2/articles/29377835.
- **More AIR-400 subjects:** S01–S10, from about 2.4 MB to 12 MB per clip.
