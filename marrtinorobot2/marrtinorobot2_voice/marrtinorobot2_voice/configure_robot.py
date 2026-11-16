#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import tempfile
from pathlib import Path

from flask import Flask, request, redirect, url_for, jsonify, render_template_string, abort

try:
    import fcntl  # Linux file locking
except ImportError:
    fcntl = None


# =========================
# Path: trova /config/*.json
# =========================

def find_config_dir(start: Path) -> Path:
    """
    Risale le directory partendo da 'start' finché trova:
      config/asr-tts.json, config/asr_tts.json oppure config/config.json
    """
    for p in [start, *start.parents]:
        cfg_dir = p / "config"
        if (
            (cfg_dir / "asr-tts.json").exists()
            or (cfg_dir / "asr_tts.json").exists()
            or (cfg_dir / "config.json").exists()
        ):
            return cfg_dir.resolve()
    # fallback: parent/config
    return (start.parent / "config").resolve()


APP_DIR = Path(__file__).resolve().parent  # .../marrtinorobot2_voice/marrtinorobot2_voice
CONFIG_DIR = Path(os.environ.get("CONFIG_DIR", find_config_dir(APP_DIR))).resolve()
ASR_TTS_CONFIG = CONFIG_DIR / "asr-tts.json"
if not ASR_TTS_CONFIG.exists():
    ASR_TTS_CONFIG = CONFIG_DIR / "asr_tts.json"

FILES = {
    "asr_tts": ASR_TTS_CONFIG,
    "config":  CONFIG_DIR / "config.json",
}


# =========================
# Flask
# =========================

app = Flask(__name__)

TEMPLATE = """
<!doctype html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Config editor</title>
  <style>
    body{font-family:system-ui,Arial;max-width:980px;margin:24px auto;padding:0 12px}
    .top{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
    .card{border:1px solid #ddd;border-radius:10px;padding:16px;margin-top:16px}
    label{display:block;font-weight:600;margin-top:10px}
    input, textarea, select{width:100%;padding:10px;border:1px solid #ccc;border-radius:8px}
    .muted{color:#666;font-size:0.95rem}
    .btn{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #222;background:#111;color:#fff;text-decoration:none}
    .btn2{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #bbb;background:#fff;color:#111;text-decoration:none}
    .ok{color:#0a7}
    .err{color:#c00}
    code{background:#f6f6f6;padding:2px 6px;border-radius:6px}
    .radio-row{display:flex;gap:10px;flex-wrap:wrap;margin:12px 0}
    .radio-card{flex:1 1 220px;border:1px solid #ddd;border-radius:10px;padding:12px}
    .radio-card input{width:auto;margin-right:8px}
  </style>
</head>
<body>
  <div class="top">
    <h2 style="margin:0">Config editor</h2>
    <form method="get" action="/" style="margin:0">
      <select name="which" onchange="this.form.submit()">
        <option value="asr_tts" {{'selected' if which=='asr_tts' else ''}}>asr_tts.json</option>
        <option value="config"  {{'selected' if which=='config'  else ''}}>config.json</option>
      </select>
    </form>
    <a class="btn2" href="/chatbot">Chatbot EVA/N8N</a>
    <a class="btn2" href="/api/config/{{which}}">API: GET JSON</a>
  </div>

  <p class="muted">Config dir: <code>{{config_dir}}</code></p>

  {% if msg %}
    <p class="{{'ok' if ok else 'err'}}">{{msg}}</p>
  {% endif %}

  <div class="card">
    <form method="post" action="/save?which={{which}}">
      {% for keypath, display_value in flat_items %}
        <label for="{{keypath}}">{{keypath}}</label>
        <input id="{{keypath}}" name="{{keypath}}" value="{{display_value|e}}">
      {% endfor %}

      <div style="margin-top:16px;display:flex;gap:10px;flex-wrap:wrap">
        <button class="btn" type="submit">Salva</button>
        <a class="btn2" href="/">Ricarica</a>
      </div>

      <p class="muted" style="margin-top:10px">
        Liste: separa con virgole (es. <code>martino, martina</code>) oppure incolla JSON (<code>["a","b"]</code>).<br>
        Bool: <code>true/false</code>.
      </p>
    </form>
  </div>
</body>
</html>
"""

CHATBOT_TEMPLATE = """
<!doctype html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Config chatbot</title>
  <style>
    body{font-family:system-ui,Arial;max-width:880px;margin:24px auto;padding:0 12px}
    .top{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
    .card{border:1px solid #ddd;border-radius:10px;padding:16px;margin-top:16px}
    label{display:block;font-weight:600;margin-top:12px}
    input{width:100%;padding:10px;border:1px solid #ccc;border-radius:8px}
    input[type=radio]{width:auto;margin-right:8px}
    .radio-row{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;margin-top:10px}
    .radio-card{border:1px solid #ddd;border-radius:10px;padding:12px}
    .muted{color:#666;font-size:0.95rem}
    .btn{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #222;background:#111;color:#fff;text-decoration:none}
    .btn2{display:inline-block;padding:10px 14px;border-radius:10px;border:1px solid #bbb;background:#fff;color:#111;text-decoration:none}
    .ok{color:#0a7}
    .err{color:#c00}
    code{background:#f6f6f6;padding:2px 6px;border-radius:6px}
  </style>
</head>
<body>
  <div class="top">
    <h2 style="margin:0">Chatbot EVA / N8N</h2>
    <a class="btn2" href="/">Editor JSON</a>
    <a class="btn2" href="/api/config/asr_tts">API config</a>
  </div>

  <p class="muted">File: <code>{{path}}</code></p>

  {% if msg %}
    <p class="{{'ok' if ok else 'err'}}">{{msg}}</p>
  {% endif %}

  <div class="card">
    <form method="post" action="/chatbot/save">
      <label>Risposte del robot</label>
      <div class="radio-row">
        <div class="radio-card">
          <label><input type="radio" name="provider" value="eva" {{'checked' if provider == 'eva' else ''}}> EVA</label>
          <p class="muted">Usa l'assistente EVA diretto.</p>
        </div>
        <div class="radio-card">
          <label><input type="radio" name="provider" value="n8n" {{'checked' if provider == 'n8n' else ''}}> N8N</label>
          <p class="muted">Usa il bridge N8N tramite `MarrtinoN8N/eva_bridge.py`.</p>
        </div>
      </div>

      <label for="eva_url">Endpoint EVA</label>
      <input id="eva_url" name="eva_url" value="{{eva_url|e}}">

      <label for="n8n_url">Endpoint N8N</label>
      <input id="n8n_url" name="n8n_url" value="{{n8n_url|e}}">

      <label for="connect_timeout">Timeout connessione</label>
      <input id="connect_timeout" name="connect_timeout" value="{{connect_timeout|e}}">

      <label for="read_timeout">Timeout risposta</label>
      <input id="read_timeout" name="read_timeout" value="{{read_timeout|e}}">

      <label for="max_retries">Tentativi extra</label>
      <input id="max_retries" name="max_retries" value="{{max_retries|e}}">

      <div style="margin-top:16px;display:flex;gap:10px;flex-wrap:wrap">
        <button class="btn" type="submit">Salva</button>
        <a class="btn2" href="/chatbot">Ricarica</a>
      </div>
    </form>
  </div>
</body>
</html>
"""


# =========================
# Helpers I/O + lock
# =========================

def _lock_file(f):
    if fcntl is None:
        return
    fcntl.flock(f.fileno(), fcntl.LOCK_EX)

def _unlock_file(f):
    if fcntl is None:
        return
    fcntl.flock(f.fileno(), fcntl.LOCK_UN)

def read_json(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def atomic_write_json(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)

    # backup best-effort
    backup = path.with_suffix(path.suffix + ".bak")
    if path.exists():
        try:
            backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
        except Exception:
            pass

    # write atomico (temp nella stessa cartella -> replace)
    tmp_fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(tmp_fd, "w", encoding="utf-8") as tmp:
            json.dump(data, tmp, ensure_ascii=False, indent=2)
            tmp.write("\n")
        os.replace(tmp_name, path)
    finally:
        try:
            if os.path.exists(tmp_name):
                os.remove(tmp_name)
        except Exception:
            pass


# =========================
# Helpers flatten + cast
# =========================

def flatten(obj, prefix=""):
    """
    Ritorna lista (keypath, value). I dict vengono espansi (beep.enabled),
    le liste restano come valore singolo (wake_words).
    """
    items = []
    if isinstance(obj, dict):
        for k in obj.keys():
            v = obj[k]
            kp = f"{prefix}.{k}" if prefix else str(k)
            items.extend(flatten(v, kp))
    else:
        items.append((prefix, obj))
    return items

def set_by_keypath(root: dict, keypath: str, value):
    parts = keypath.split(".")
    cur = root
    for p in parts[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            cur[p] = {}
        cur = cur[p]
    cur[parts[-1]] = value

def get_by_keypath(root: dict, keypath: str):
    cur = root
    for p in keypath.split("."):
        if not isinstance(cur, dict) or p not in cur:
            raise KeyError(keypath)
        cur = cur[p]
    return cur

def parse_bool(s: str) -> bool:
    s = s.strip().lower()
    if s in ("true", "1", "yes", "y", "on"):
        return True
    if s in ("false", "0", "no", "n", "off"):
        return False
    raise ValueError("bool non valido (usa true/false)")

def cast_like(s: str, prototype):
    """
    Converte la stringa s mantenendo il tipo del valore esistente (prototype).
    """
    if prototype is None:
        return s

    if isinstance(prototype, bool):
        return parse_bool(s)

    if isinstance(prototype, int) and not isinstance(prototype, bool):
        return int(s.strip())

    if isinstance(prototype, float):
        return float(s.strip())

    if isinstance(prototype, list):
        raw = s.strip()

        # se l’utente incolla proprio JSON
        if raw.startswith("[") and raw.endswith("]"):
            v = json.loads(raw)
            if not isinstance(v, list):
                raise ValueError("atteso lista")
            return v

        parts = [p.strip() for p in raw.split(",") if p.strip() != ""]

        # prova a castare gli elementi come il primo della lista esistente
        if prototype:
            return [cast_like(p, prototype[0]) for p in parts]
        return parts

    if isinstance(prototype, dict):
        raw = s.strip()
        v = json.loads(raw)
        if not isinstance(v, dict):
            raise ValueError("atteso oggetto JSON")
        return v

    return s  # string (o altri tipi)


# =========================
# Caricamento config
# =========================

def load_config(which: str):
    if which not in FILES:
        abort(404)
    path = FILES[which]
    if not path.exists():
        abort(404, description=f"File non trovato: {path}")
    data = read_json(path)
    return path, data


# =========================
# Routes
# =========================

@app.get("/")
def index():
    which = request.args.get("which", "asr_tts")
    _, data = load_config(which)

    flat = flatten(data)
    # ordine stabile in UI
    flat = sorted(flat, key=lambda x: x[0])

    flat_items = []
    for kp, v in flat:
        if isinstance(v, list):
            flat_items.append((kp, ", ".join(str(x) for x in v)))
        else:
            flat_items.append((kp, str(v)))

    msg = request.args.get("msg")
    ok = request.args.get("ok") == "1"

    return render_template_string(
        TEMPLATE,
        which=which,
        config_dir=str(CONFIG_DIR),
        flat_items=flat_items,
        msg=msg,
        ok=ok,
    )

@app.post("/save")
def save_from_form():
    which = request.args.get("which", "asr_tts")
    path, _ = load_config(which)

    try:
        # prova lock sul file (se permessi ok)
        lockf = open(path, "a+", encoding="utf-8")
    except Exception as e:
        return redirect(url_for("index", which=which, msg=f"Errore permessi su {path}: {e}", ok="0"))

    try:
        _lock_file(lockf)
        current = read_json(path)  # rileggi sotto lock

        # per ogni campo esistente, cast coerente con il tipo attuale
        for kp, proto in flatten(current):
            if kp in request.form:
                raw = request.form[kp]
                new_val = cast_like(raw, proto)
                set_by_keypath(current, kp, new_val)

        atomic_write_json(path, current)

    except Exception as e:
        return redirect(url_for("index", which=which, msg=f"Errore salvataggio: {e}", ok="0"))
    finally:
        try:
            _unlock_file(lockf)
            lockf.close()
        except Exception:
            pass

    return redirect(url_for("index", which=which, msg=f"Salvato: {path.name}", ok="1"))

@app.get("/chatbot")
def chatbot_page():
    path, data = load_config("asr_tts")
    chatbot = data.get("chatbot", {})
    if not isinstance(chatbot, dict):
        chatbot = {}
    endpoints = chatbot.get("endpoints", {})
    if not isinstance(endpoints, dict):
        endpoints = {}

    provider = str(chatbot.get("provider", "eva")).lower().strip()
    if provider not in ("eva", "n8n"):
        provider = "eva"

    return render_template_string(
        CHATBOT_TEMPLATE,
        path=str(path),
        provider=provider,
        eva_url=endpoints.get("eva", "http://127.0.0.1:5000/json"),
        n8n_url=endpoints.get("n8n", "http://127.0.0.1:5002/json"),
        connect_timeout=chatbot.get("connect_timeout", 3),
        read_timeout=chatbot.get("read_timeout", 45),
        max_retries=chatbot.get("max_retries", 2),
        msg=request.args.get("msg"),
        ok=request.args.get("ok") == "1",
    )

@app.post("/chatbot/save")
def chatbot_save():
    path, _ = load_config("asr_tts")

    try:
        lockf = open(path, "a+", encoding="utf-8")
    except Exception as e:
        return redirect(url_for("chatbot_page", msg=f"Errore permessi su {path}: {e}", ok="0"))

    try:
        provider = request.form.get("provider", "eva").lower().strip()
        if provider not in ("eva", "n8n"):
            provider = "eva"

        connect_timeout = float(request.form.get("connect_timeout", "3"))
        read_timeout = float(request.form.get("read_timeout", "45"))
        max_retries = int(request.form.get("max_retries", "2"))

        eva_url = request.form.get("eva_url", "").strip() or "http://127.0.0.1:5000/json"
        n8n_url = request.form.get("n8n_url", "").strip() or "http://127.0.0.1:5002/json"

        _lock_file(lockf)
        current = read_json(path)
        current["chatbot"] = {
            "provider": provider,
            "endpoints": {
                "eva": eva_url,
                "n8n": n8n_url,
            },
            "connect_timeout": connect_timeout,
            "read_timeout": read_timeout,
            "max_retries": max_retries,
        }
        atomic_write_json(path, current)
    except Exception as e:
        return redirect(url_for("chatbot_page", msg=f"Errore salvataggio: {e}", ok="0"))
    finally:
        try:
            _unlock_file(lockf)
            lockf.close()
        except Exception:
            pass

    return redirect(url_for("chatbot_page", msg=f"Salvato: {path.name}", ok="1"))

@app.get("/api/config/<which>")
def api_get(which):
    _, data = load_config(which)
    return jsonify(data)

@app.post("/api/config/<which>")
def api_patch(which):
    path, _ = load_config(which)
    patch = request.get_json(silent=True)
    if not isinstance(patch, dict):
        return jsonify({"ok": False, "error": "Body JSON deve essere un oggetto"}), 400

    try:
        lockf = open(path, "a+", encoding="utf-8")
    except Exception as e:
        return jsonify({"ok": False, "error": f"Permessi su {path}: {e}"}), 403

    try:
        _lock_file(lockf)
        current = read_json(path)

        # patch: supporta anche keypath tipo "beep.enabled"
        for k, v in patch.items():
            if "." in k:
                try:
                    proto = get_by_keypath(current, k)
                    if isinstance(v, str):
                        v = cast_like(v, proto)
                except Exception:
                    pass
                set_by_keypath(current, k, v)
            else:
                if k in current and isinstance(v, str):
                    v = cast_like(v, current[k])
                current[k] = v

        atomic_write_json(path, current)
        return jsonify({"ok": True, "file": str(path), "config_dir": str(CONFIG_DIR)})

    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    finally:
        try:
            _unlock_file(lockf)
            lockf.close()
        except Exception:
            pass


# =========================
# Main
# =========================

if __name__ == "__main__":
    # Porta richiesta: 8081
    app.run(host="0.0.0.0", port=8081, debug=True)
