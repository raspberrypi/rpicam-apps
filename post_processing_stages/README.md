## AcousticFocusStage – Documentation (English)

**Purpose:**  
The AcousticFocusStage is a post-processing stage for rpicam-apps that generates an audible tone based on the libcamera Focus Figure of Merit (FoM).  
This helps you find the optimal focus point of a manual lens **without needing to look at or interpret the preview image**.

### Features

- Plays a sine tone via the Raspberry Pi’s audio output.
- The tone’s frequency is proportional to the current Focus FoM value.
- The tone is triggered once per second for 0.1 seconds (adjustable).
- Useful for manual focusing when the preview is not visible or practical.

### Dependencies

Install the following packages:

```sh
sudo apt update
sudo apt install sox libsox-fmt-all
```

### Build Instructions

1. Add acoustic_focus_stage.cpp to your post_processing_stages directory.
2. Add the stage to your meson.build:
   ```meson
   core_postproc_src = files([
       ...
       'acoustic_focus_stage.cpp',
   ])
   postproc_assets += files([
       ...
       assets_dir / 'acoustic_focus.json',
   ])
   ```
3. Rebuild and install:
   ```sh
   ninja
   sudo ninja install
   ```

### Usage

1. Create a config file acoustic_focus.json:
   ```json
   {
     "post_process": [
       { "stage": "acoustic_focus" }
     ]
   }
   ```
2. Start rpicam-vid with:
   ```sh
   rpicam-vid --post-process-config assets/acoustic_focus.json
   ```
3. Adjust focus on your manual lens. The tone’s pitch will rise as the focus improves.

---

## AcousticFocusStage – Dokumentation (Deutsch)

**Zweck:**  
Die AcousticFocusStage ist eine Post-Processing-Stage für rpicam-apps, die einen Ton ausgibt, dessen Frequenz vom libcamera Focus Figure of Merit (FoM) abhängt.  
Damit kannst du den optimalen Fokuspunkt einer manuellen Linse **finden, ohne auf die Vorschau schauen oder diese interpretieren zu müssen**.

### Funktionen

- Gibt einen Sinuston über den Audio-Ausgang des Raspberry Pi aus.
- Die Tonhöhe entspricht dem aktuellen Focus FoM-Wert.
- Der Ton wird einmal pro Sekunde für 0,1 Sekunden (anpassbar) ausgegeben.
- Ideal für das manuelle Fokussieren, wenn kein Blickkontakt zur Vorschau möglich ist.

### Abhängigkeiten

Installiere folgende Pakete:

```sh
sudo apt update
sudo apt install sox libsox-fmt-all
```

### Kompilierung

1. Lege acoustic_focus_stage.cpp im Verzeichnis post_processing_stages ab.
2. Ergänze die Stage in deiner meson.build:
   ```meson
   core_postproc_src = files([
       ...
       'acoustic_focus_stage.cpp',
   ])
   postproc_assets += files([
       ...
       assets_dir / 'acoustic_focus.json',
   ])
   ```
3. Baue und installiere neu:
   ```sh
   ninja
   sudo ninja install
   ```

### Verwendung

1. Erstelle die Konfigurationsdatei acoustic_focus.json:
   ```json
   {
     "post_process": [
       { "stage": "acoustic_focus" }
     ]
   }
   ```
2. Starte rpicam-vid mit:
   ```sh
   rpicam-vid --post-process-config assets/acoustic_focus.json
   ```
3. Drehe am Fokusring deiner manuellen Linse. Die Tonhöhe steigt, je besser der Fokus ist.

---

**Hinweis:**  
Die Stage ist ein reines Hilfsmittel für das manuelle Fokussieren und benötigt keinen Blickkontakt zum Monitor!