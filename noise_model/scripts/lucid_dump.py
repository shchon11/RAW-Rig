"""Read-only full nodemap dump of a LUCID camera (changes nothing).

  python lucid_dump.py [--serial S] [--out data/]
"""
import argparse
import json
import time
from pathlib import Path

KEY = ("HDR", "PixelFormat", "Exposure", "Gain", "BlackLevel", "Balance", "Gamma", "LUT", "Tone",
       "AcquisitionFrameRate", "DeviceTemperature", "ConversionGain", "ADC", "Width", "Height", "Binning",
       "Defect", "ColorTransformation", "Isp", "ISP", "Clamp", "Shading", "Flicker")


def describe(n):
    d = {"type": str(getattr(n, "interface_type", "")).split(".")[-1], "access": str(n.access_mode).split(".")[-1]}
    if n.is_readable:
        try:
            v = n.value
            d["value"] = v if isinstance(v, (int, float, bool)) or v is None else str(v)
        except Exception as e:
            d["value_error"] = str(e)
        for a in ("min", "max", "inc", "unit"):
            try:
                v = getattr(n, a)
                if v not in (None, ""):
                    d[a] = v if isinstance(v, (int, float)) else str(v)
            except Exception:
                pass
        try:
            d["entries"] = list(n.enumentry_names)
        except Exception:
            pass
    try:
        d["description"] = n.description
    except Exception:
        pass
    return d


def dump_map(nm):
    out = {}
    for name in nm.feature_names:
        try:
            out[name] = describe(nm.get_node(name))
        except Exception as e:
            out[name] = {"error": str(e)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent.parent / "data")
    a = ap.parse_args()
    from arena_api.system import system
    infos = [i for i in system.device_infos if i.get("vendor", "").startswith("Lucid")]
    if a.serial:
        infos = [i for i in infos if i["serial"] == a.serial]
    if not infos:
        raise SystemExit("LUCID camera not found")
    dev = system.create_device(device_infos=[infos[0]])[0]
    try:
        report = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "device_info": {k: str(v) for k, v in infos[0].items()},
                  "nodemap": dump_map(dev.nodemap), "tl_stream": dump_map(dev.tl_stream_nodemap),
                  "tl_device": dump_map(dev.tl_device_nodemap)}
    finally:
        system.destroy_device()
    a.out.mkdir(parents=True, exist_ok=True)
    path = a.out / f"nodemap_{infos[0]['serial']}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str))
    nm = report["nodemap"]
    print(f"{len(nm)} nodes -> {path}")
    for name, d in nm.items():
        if any(k.lower() in name.lower() for k in KEY) and d.get("type") not in ("ICategory",):
            extra = {k: d[k] for k in ("min", "max", "inc", "unit", "entries") if k in d}
            print(f"  {name:45s} {d.get('access',''):3s} {str(d.get('value', ''))[:40]:40s} {extra if extra else ''}")


if __name__ == "__main__":
    main()
