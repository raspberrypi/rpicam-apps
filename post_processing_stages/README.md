## AcousticFocusStage – Documentation (English)

**Purpose:**  
The AcousticFocusStage is a post-processing stage for rpicam-apps that provides acoustic feedback based on the libcamera Focus Figure of Merit (FoM).  
This allows you to find the optimal focus point of a manual lens **without needing to look at or interpret the preview image**.

### Features

- Plays a sine tone via the Raspberry Pi’s audio output.
- The tone’s frequency is mapped to the current Focus FoM value (frequency rises or falls as FoM rises or falls).
- No visual contact with the preview is required.
- The tone is triggered once per second for a configurable duration.
- All parameters (frequency range, mapping type, duration, etc.) can be configured via JSON.
- **Note:** You must use an external USB sound card, HDMI audio, or another supported audio device for this stage to function.

### Dependencies

Install the following packages:

```sh
sudo apt update
sudo apt install sox libsox-fmt-all
```

### Build Instructions

1. Add `acoustic_focus_stage.cpp` to your `post_processing_stages` directory.
2. Add the stage to your `meson.build`:
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
   meson compile -C build
   sudo meson install -C build
   ```

### Configuration

Create a config file `acoustic_focus.json` (example):

```json
{
  "acoustic_focus": [
    {
      "stage": "acoustic_focus",
      "minFoM": 1,
      "maxFoM": 2000,
      "minFreq": 300,
      "maxFreq": 5000,
      "duration": 0.1,
      "mapping": "log",
      "description": "mapping values are log (logarithmic) or linear"
    }
  ]
}
```

- `minFoM`, `maxFoM`: Range of Figure of Merit values to map.
- `minFreq`, `maxFreq`: Frequency range for the output tone (Hz).
- `duration`: Tone duration in seconds.
- `mapping`: `"log"` for logarithmic mapping, `"linear"` for linear mapping.

### Usage

1. Start rpicam-vid with:
   ```sh
   rpicam-vid --post-process-config assets/acoustic_focus.json
   ```
2. Adjust focus on your manual lens. The tone’s pitch will rise or fall as the focus improves or worsens.

---

## AcousticFocusStage – Dokumentation (Deutsch)

**Zweck:**  
Die AcousticFocusStage ist eine Post-Processing-Stage für rpicam-apps, die akustisches Feedback auf Basis des libcamera Focus Figure of Merit (FoM) gibt.  
Damit findest du den optimalen Fokuspunkt einer manuellen Linse **ohne auf die Vorschau schauen oder diese interpretieren zu müssen**.

### Funktionen

- Gibt einen Sinuston über den Audio-Ausgang des Raspberry Pi aus.
- Die Tonhöhe wird aus dem aktuellen Focus FoM-Wert berechnet (steigt oder fällt mit dem FoM).
- Kein Sichtkontakt zur Vorschau erforderlich.
- Der Ton wird einmal pro Sekunde für eine konfigurierbare Dauer ausgegeben.
- Alle Parameter (Frequenzbereich, Mapping-Typ, Dauer usw.) sind per JSON konfigurierbar.
- **Hinweis:** Es muss eine Soundausgabe-Hardware vorhanden sein.

### Abhängigkeiten

Installiere folgende Pakete:

```sh
sudo apt update
sudo apt install sox libsox-fmt-all
```

### Kompilierung

1. Lege `acoustic_focus_stage.cpp` im Verzeichnis `post_processing_stages` ab.
2. Ergänze die Stage in deiner `meson.build`:
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
   meson compile -C build
   sudo meson install -C build
   ```

### Konfiguration

Beispiel für `acoustic_focus.json`:

```json
{
  "acoustic_focus": [
    {
      "stage": "acoustic_focus",
      "minFoM": 1,
      "maxFoM": 2000,
      "minFreq": 400,
      "maxFreq": 2000,
      "duration": 0.1,
      "mapping": "log", // oder "linear"
      "description": "mapping values are log (logarithmic) or linear"
    }
  ]
}
```

- `minFoM`, `maxFoM`: Bereich der Figure of Merit-Werte.
- `minFreq`, `maxFreq`: Frequenzbereich für den Ton (Hz).
- `duration`: Tondauer in Sekunden.
- `mapping`: `"log"` für logarithmisch, `"linear"` für linear.

### Verwendung

1. Starte rpicam-vid mit:
   ```sh
   rpicam-vid --post-process-config assets/acoustic_focus.json
   ```
2. Drehe am Fokusring deiner manuellen Linse. Die Tonhöhe steigt oder fällt, je nach Fokusqualität.

---

**Hinweis:**  
Die Stage ist ein reines Hilfsmittel für das manuelle Fokussieren und benötigt keinen Blickkontakt zum Monitor!