# Object Blur Stage - Documentation / Dokumentation

**English** | [Deutsch](#deutsch)

---

## <a name="english"></a>English

### Overview

The **object_blur** stage enables automatic blurring of detected objects in the video stream. This is useful for privacy applications or to obscure specific objects in real-time.

### Dependencies

**Important**: This stage is **not standalone** and requires a previous object detection stage to function.

**Required**:
- An object detection stage like `hailo_yolo_inference` must be configured **before** `object_blur` in your JSON
- The detection stage must provide bounding boxes via `object_detect.results`

**Example minimal working configuration**:
```json
{
    "hailo_yolo_inference": {
        "hef_file_8": "/usr/share/hailo-models/yolov8s_h8.hef",
        "max_detections": 20,
        "threshold": 0.4
    },
    "object_blur": {
        "overlay_blur": ["person"]
    }
}
```

Without a detection stage, `object_blur` will have no objects to blur and will do nothing.

### How it works

The stage reads object detections from previous stages (e.g., `hailo_yolo_inference`) and applies various blur effects to the detected objects. Objects to be blurred are specified via their COCO class names (e.g., "person", "cup", "wine glass").

**Pro Tip**: You can record "normal" videos with active inference but without bounding boxes by omitting `object_detect_draw_cv` from your configuration. This creates a clean, professional look where objects are blurred **live** without visible detection frames - perfect for privacy-focused **live** recordings! There's no need for time-consuming post-processing video editing...

### Table of Contents

- [Overview](#overview)
- [Dependencies](#dependencies)
- [How it works](#how-it-works)
- [JSON Configuration](#json-configuration)
- [Minimal Configuration](#minimal-configuration)
- [Full Configuration](#full-configuration)
- [Parameter Reference](#parameter-reference)
- [Usage Examples](#usage-examples)
- [COCO Class Names](#coco-class-names)
- [Performance Tips](#performance-tips)
- [Installation](#installation)
- [Troubleshooting](#troubleshooting)

### <a name="json-configuration"></a>JSON Configuration

#### <a name="minimal-configuration"></a>Minimal Configuration

```json
{
    "object_blur": {
        "overlay_blur": ["person", "cup", "wine glass"]
    }
}
```

#### <a name="full-configuration"></a>Full Configuration with all Parameters

```json
{
    "object_blur": {
        "overlay_blur": ["person", "cup", "wine glass", "bottle"],
        "blur_type": "pixelate",
        "blur_strength": 16,
        "gaussian_sigma": 0,
        "expand_box": false,
        "expand_pixels": 0,
        "_info": {
            "description": "Blur detected objects based on COCO class names",
            "overlay_blur": "Array of COCO class names to blur (required)",
            "blur_type": "Blur method: 'pixelate' (default, fastest), 'gaussian' (smooth), 'median' (noise reduction)",
            "blur_strength": "Blur intensity: 0=auto, pixelate:8-32, gaussian/median:15-51 (must be odd)",
            "gaussian_sigma": "Gaussian blur sigma: 0=auto (kernel_size/6), higher=stronger blur",
            "expand_box": "Expand bounding box before blur: true/false",
            "expand_pixels": "Pixels to expand box: 0=auto 10% if expand_box=true, or fixed pixel value"
        }
    }
}
```

### <a name="parameter-reference"></a>Parameter Reference

#### `overlay_blur` (required)
- **Type**: Array of Strings
- **Description**: List of COCO class names to be blurred
- **Examples**: 
  - `["person"]` - blur persons only
  - `["wine glass", "cup", "bottle"]` - beverage-related objects only
  - `["cell phone", "laptop"]` - electronic devices

#### `blur_type` (optional)
- **Type**: String
- **Default**: `"pixelate"`
- **Possible Values**:
  - `"pixelate"` - Pixelation effect (Minecraft-like look, fastest)
  - `"gaussian"` - Smooth Gaussian blur (camera-like blur)
  - `"median"` - Median blur (good for noise reduction, medium speed)

#### `blur_strength` (optional)
- **Type**: Integer
- **Default**: `0` (automatically calculated based on object size)
- **Description**: 
  - For `pixelate`: Block size in pixels (larger values = stronger pixelation)
    - Recommended values: 8, 12, 16, 20, 24, 32
  - For `gaussian`/`median`: Kernel size (must be odd)
    - Recommended values: 15, 21, 31, 41, 51
- **Examples**:
  - `blur_strength: 8` - light pixelation
  - `blur_strength: 32` - strong pixelation
  - `blur_strength: 51` - very strong Gaussian blur

#### `gaussian_sigma` (optional)
- **Type**: Integer
- **Default**: `0` (automatically calculated as kernel_size / 6)
- **Description**: Only relevant for `blur_type: "gaussian"`. Higher values = stronger blur effect
- **Recommended values**: 0 (auto), 5, 10, 15, 20

#### `expand_box` (optional)
- **Type**: Boolean
- **Default**: `false`
- **Description**: Expands the bounding box before blurring
  - `false` - only exact detection box is blurred
  - `true` - box is expanded by 10% or by `expand_pixels`

#### `expand_pixels` (optional)
- **Type**: Integer
- **Default**: `0`
- **Description**: Number of pixels to expand the bounding box in all directions
- **Examples**:
  - `expand_pixels: 10` - 10 pixel border around each object
  - `expand_pixels: 20` - 20 pixel border (useful for completely covering faces)
  - `expand_pixels: 0` with `expand_box: true` - automatically 10% of box size

### <a name="usage-examples"></a>Usage Examples

#### Example 1: Pixelate persons (Privacy)

```json
{
    "hailo_yolo_inference": {
        "hef_file_8": "/usr/share/hailo-models/yolov8s_h8.hef",
        "max_detections": 20,
        "threshold": 0.4
    },
    "object_blur": {
        "overlay_blur": ["person"],
        "blur_type": "pixelate",
        "blur_strength": 20,
        "expand_box": true,
        "expand_pixels": 15
    }
}
```

#### Example 2: Clean Recording without Bounding Boxes

Record a "normal" video with live object blurring but no visible detection frames:

```json
{
    "hailo_yolo_inference": { ... },
    "object_blur": {
        "overlay_blur": ["person", "cell phone"],
        "blur_type": "gaussian",
        "blur_strength": 31
    }
}
```

Note: Simply omit `object_detect_draw_cv` to record without bounding boxes!

#### Example 3: Blur Screens and Displays

```json
{
    "hailo_yolo_inference": { ... },
    "object_blur": {
        "overlay_blur": ["tv", "laptop", "cell phone"],
        "blur_type": "median",
        "blur_strength": 21,
        "expand_pixels": 10
    }
}
```

### <a name="coco-class-names"></a>COCO Class Names

Commonly used COCO class names (depends on YOLO model):
- Persons: `person`
- Vehicles: `car`, `truck`, `bus`, `motorcycle`, `bicycle`
- Electronics: `tv`, `laptop`, `cell phone`, `keyboard`, `mouse`
- Beverages: `bottle`, `wine glass`, `cup`
- Food: `banana`, `apple`, `sandwich`, `pizza`, `donut`, `cake`
- Furniture: `chair`, `couch`, `bed`, `dining table`
- Animals: `cat`, `dog`, `bird`, `horse`, `sheep`, `cow`

See `assets/coco.names` for the complete list of available classes.

### <a name="performance-tips"></a>Performance Tips

1. **Pixelate** is fastest and sufficient for most applications
2. **Gaussian** is slower but looks more natural
3. **Median** is slowest but good for noise reduction
4. Smaller `blur_strength` values are faster
5. `expand_box` without `expand_pixels` is faster than with fixed pixel values

### <a name="installation"></a>Installation

The stage is included in rpicam-apps by default. After compilation it's automatically available:

```bash
cd /home/admin/rpicam-apps
meson compile -C build
sudo meson install -C build
```

### Usage

```bash
rpicam-vid --post-process-file assets/hailo_yolov8_inference.json --timeout 10000
```

**Note**: You need a JSON configuration that includes both the detection stage (e.g., `hailo_yolo_inference`) AND the `object_blur` stage. A minimal example would combine detection with blur - see the configuration examples above.

### <a name="troubleshooting"></a>Troubleshooting

**Problem**: Objects are not being blurred
- Check if `hailo_yolo_inference` stage is defined before in the JSON
- Check if object names are spelled correctly (case-sensitive)
- Increase logging level to see which objects are being detected

**Problem**: Blur is too weak
- Increase `blur_strength` to a higher value
- Enable `expand_box: true` or set `expand_pixels` to a higher value

**Problem**: Performance issues
- Switch from `gaussian` to `pixelate`
- Reduce `blur_strength`
- Reduce the number of object classes to blur

---

## <a name="deutsch"></a>Deutsch

### Übersicht

Die **object_blur** Stage ermöglicht es, erkannte Objekte im Video-Stream automatisch zu blurren (verwischen/verpixeln). Dies ist nützlich für Datenschutz-Anwendungen oder um bestimmte Objekte in Echtzeit zu verbergen.
### Abhängigkeiten

**Wichtig**: Diese Stage ist **nicht eigenständig** und benötigt eine vorherige Objekterkennungs-Stage.

**Erforderlich**:
- Eine Objekterkennungs-Stage wie `hailo_yolo_inference` muss **vor** `object_blur` in der JSON konfiguriert sein
- Die Erkennungs-Stage muss Bounding Boxes über `object_detect.results` bereitstellen

**Beispiel minimal funktionierende Konfiguration**:
```json
{
    "hailo_yolo_inference": {
        "hef_file_8": "/usr/share/hailo-models/yolov8s_h8.hef",
        "max_detections": 20,
        "threshold": 0.4
    },
    "object_blur": {
        "overlay_blur": ["person"]
    }
}
```

Ohne Erkennungs-Stage hat `object_blur` keine Objekte zum Blurren und macht nichts.
## Funktionsweise

Die Stage liest Objekterkennungen aus vorherigen Stages (z.B. `hailo_yolo_inference`) und wendet verschiedene Blur-Effekte auf die erkannten Objekte an. Die zu blurrenden Objekte werden über ihre COCO-Klassennamen (z.B. "person", "cup", "wine glass") spezifiziert.

**Pro-Tipp**: Man kann "normale" Videos mit aktiver Inferenz aber ohne Bounding Boxes aufnehmen, indem man `object_detect_draw_cv` aus der Konfiguration weglässt. Das erzeugt einen sauberen, professionellen Look, bei dem Objekte **live** geblurred werden, ohne sichtbare Erkennungs-Rahmen - perfekt für datenschutzorientierte **Live**-Aufnahmen! Es entfällt auch die Notwendigkeit einer anschließenden aufwendigen Videobearbeitung...

### Inhaltsverzeichnis

- [Übersicht](#übersicht)
- [Abhängigkeiten](#abhängigkeiten)
- [Funktionsweise](#funktionsweise)
- [JSON-Konfiguration](#json-konfiguration)
- [Minimale Konfiguration](#minimale-konfiguration)
- [Vollständige Konfiguration](#vollständige-konfiguration)
- [Parameter-Referenz](#parameter-referenz)
- [Verwendungsbeispiele](#verwendungsbeispiele)
- [COCO-Klassennamen](#coco-klassennamen)
- [Performance-Tipps](#performance-tipps-de)
- [Installation](#installation-de)
- [Troubleshooting](#troubleshooting-de)

### <a name="json-konfiguration"></a>JSON-Konfiguration

#### <a name="minimale-konfiguration"></a>Minimale Konfiguration

```json
{
    "object_blur": {
        "overlay_blur": ["person", "car", "wine glass"]
    }
}
```

### Vollständige Konfiguration mit allen Parametern

```json
{
    "object_blur": {
        "overlay_blur": ["person", "cup", "wine glass", "bottle"],
        "blur_type": "pixelate",
        "blur_strength": 16,
        "gaussian_sigma": 0,
        "expand_box": false,
        "expand_pixels": 0
    }
}
```

### <a name="parameter-referenz"></a>Parameter-Referenz

#### `overlay_blur` (erforderlich)
- **Typ**: Array von Strings
- **Beschreibung**: Liste der COCO-Klassennamen, die geblurred werden sollen
- **Beispiele**: 
  - `["person"]` - nur Personen blurren
  - `["wine glass", "cup", "bottle"]` - nur Getränke-bezogene Objekte
  - `["cell phone", "laptop"]` - elektronische Geräte

#### `blur_type` (optional)
- **Typ**: String
- **Standard**: `"pixelate"`
- **Mögliche Werte**:
  - `"pixelate"` - Pixelierungs-Effekt (wie Minecraft-Look, am schnellsten)
  - `"gaussian"` - Weicher Gaussian-Blur (wie Kamera-Unschärfe)
  - `"median"` - Median-Blur (gut gegen Rauschen, mittlere Geschwindigkeit)

#### `blur_strength` (optional)
- **Typ**: Integer
- **Standard**: `0` (automatisch berechnet basierend auf Objektgröße)
- **Beschreibung**: 
  - Für `pixelate`: Blockgröße in Pixeln (größere Werte = stärkere Verpixelung)
    - Empfohlene Werte: 8, 12, 16, 20, 24, 32
  - Für `gaussian`/`median`: Kernel-Größe (muss ungerade sein)
    - Empfohlene Werte: 15, 21, 31, 41, 51
- **Beispiele**:
  - `blur_strength: 8` - leichte Pixelierung
  - `blur_strength: 32` - starke Pixelierung
  - `blur_strength: 51` - sehr starker Gaussian-Blur

#### `gaussian_sigma` (optional)
- **Typ**: Integer
- **Standard**: `0` (automatisch berechnet als kernel_size / 6)
- **Beschreibung**: Nur relevant für `blur_type: "gaussian"`. Höhere Werte = stärkerer Blur-Effekt
- **Empfohlene Werte**: 0 (auto), 5, 10, 15, 20

#### `expand_box` (optional)
- **Typ**: Boolean
- **Standard**: `false`
- **Beschreibung**: Erweitert die Bounding Box vor dem Blurren
  - `false` - nur exakte Detection-Box wird geblurred
  - `true` - Box wird um 10% oder um `expand_pixels` erweitert

#### `expand_pixels` (optional)
- **Typ**: Integer
- **Standard**: `0`
- **Beschreibung**: Anzahl der Pixel, um die die Bounding Box in alle Richtungen erweitert wird
- **Beispiele**:
  - `expand_pixels: 10` - 10 Pixel Rand um jedes Objekt
  - `expand_pixels: 20` - 20 Pixel Rand (nützlich um z.B. Gesichter komplett zu verdecken)
  - `expand_pixels: 0` mit `expand_box: true` - automatisch 10% der Box-Größe

### <a name="verwendungsbeispiele"></a>Verwendungsbeispiele

#### Beispiel 1: Personen verpixeln (Privacy)

```json
{
    "hailo_yolo_inference": {
        "hef_file_8": "/usr/share/hailo-models/yolov8s_h8.hef",
        "max_detections": 20,
        "threshold": 0.4
    },
    "object_blur": {
        "overlay_blur": ["person"],
        "blur_type": "pixelate",
        "blur_strength": 20,
        "expand_box": true,
        "expand_pixels": 15
    }
}
```

#### Beispiel 2: Saubere Aufnahme ohne Bounding Boxes

Ein "normales" Video mit Live-Objekt-Blurring aber ohne sichtbare Erkennungs-Rahmen aufnehmen:

```json
{
    "hailo_yolo_inference": { ... },
    "object_blur": {
        "overlay_blur": ["person", "cell phone"],
        "blur_type": "gaussian",
        "blur_strength": 31
    }
}
```

Hinweis: Einfach `object_detect_draw_cv` weglassen, um ohne Bounding Boxes aufzuzeichnen!

#### Beispiel 3: Bildschirme und Displays unkenntlich machen

```json
{
    "hailo_yolo_inference": { ... },
    "object_blur": {
        "overlay_blur": ["tv", "laptop", "cell phone"],
        "blur_type": "median",
        "blur_strength": 21,
        "expand_pixels": 10
    }
}
```

### <a name="coco-klassennamen"></a>COCO-Klassennamen

Häufig verwendete COCO-Klassennamen (je nach verwendetem YOLO-Modell):
- Personen: `person`
- Fahrzeuge: `car`, `truck`, `bus`, `motorcycle`, `bicycle`
- Elektronik: `tv`, `laptop`, `cell phone`, `keyboard`, `mouse`
- Getränke: `bottle`, `wine glass`, `cup`
- Essen: `banana`, `apple`, `sandwich`, `pizza`, `donut`, `cake`
- Möbel: `chair`, `couch`, `bed`, `dining table`
- Tiere: `cat`, `dog`, `bird`, `horse`, `sheep`, `cow`

Die vollständige Liste hängt vom verwendeten YOLO-Modell ab. Siehe `assets/coco.names` für alle verfügbaren Klassen.

### <a name="performance-tipps-de"></a>Performance-Tipps

1. **Pixelate** ist am schnellsten und ausreichend für die meisten Anwendungen
2. **Gaussian** ist langsamer, aber sieht natürlicher aus
3. **Median** ist am langsamsten, aber gut gegen Rauschen
4. Kleinere `blur_strength` Werte sind schneller
5. `expand_box` ohne `expand_pixels` ist schneller als mit festen Pixel-Werten

### <a name="installation-de"></a>Installation

Die Stage ist standardmäßig in rpicam-apps enthalten. Nach dem Kompilieren steht sie automatisch zur Verfügung:

```bash
cd /home/admin/rpicam-apps
meson compile -C build
sudo meson install -C build
```

### Verwendung

```bash
rpicam-vid --post-process-file assets/hailo_yolov8_inference.json --timeout 10000
```

**Hinweis**: Du benötigst eine JSON-Konfiguration, die sowohl die Erkennungs-Stage (z.B. `hailo_yolo_inference`) ALS AUCH die `object_blur` Stage enthält. Ein minimales Beispiel würde Detection mit Blur kombinieren - siehe die Konfigurations-Beispiele oben.

### <a name="troubleshooting-de"></a>Troubleshooting

**Problem**: Objekte werden nicht geblurred
- Prüfe, ob die `hailo_yolo_inference` Stage vorher in der JSON definiert ist
- Prüfe, ob die Objektnamen korrekt geschrieben sind (case-sensitive)
- Erhöhe die Logging-Level um zu sehen, welche Objekte erkannt werden

**Problem**: Blur ist zu schwach
- Erhöhe `blur_strength` auf einen höheren Wert
- Aktiviere `expand_box: true` oder setze `expand_pixels` auf einen höheren Wert

**Problem**: Performance-Probleme
- Wechsle von `gaussian` zu `pixelate`
- Reduziere `blur_strength`
- Reduziere die Anzahl der zu blurrenden Objektklassen

## Autor

Copyright (C) 2026, Kletternaut
