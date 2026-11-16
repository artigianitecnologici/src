"""Client HTTP delle API di Sonia Lite Web, senza tunnel o WebSocket."""
from array import array
from html.parser import HTMLParser
import io
import json
import sys
import threading
import time
from urllib import error, parse, request
import wave


def normalize_url(value):
    p = parse.urlsplit(value.strip())
    if p.scheme not in ('http', 'https') or not p.hostname or p.username or p.password:
        raise ValueError('Usa un URL come http://192.168.1.199:8096')
    try:
        p.port
    except ValueError:
        raise ValueError('Porta non valida') from None
    if p.path not in ('', '/') or p.query or p.fragment:
        raise ValueError('Indica la radice del server Sonia, senza percorso o query')
    if p.hostname in ('0.0.0.0', '::'):
        raise ValueError('Usa l’IP del server, non l’indirizzo di ascolto 0.0.0.0')
    return parse.urlunsplit((p.scheme, p.netloc, '', '', ''))


class TokenParser(HTMLParser):
    token = ''

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'meta' and attrs.get('name') == 'sonia-token':
            self.token = attrs.get('content', '')


class SoniaConnection:
    def __init__(self, emit, playback, microphone, emit_session=None):
        self.emit = emit
        self.emit_session = emit_session
        self.playback = playback
        self.microphone = microphone
        self.connected = False
        self.busy = False
        self.tts_until = 0.0
        self.transcribing = False
        self.generation = 0
        self.session = None
        # Il traffico LAN non deve passare da eventuali proxy HTTP del PC.
        self.opener = request.build_opener(request.ProxyHandler({}))

    def _emit(self, session, kind, value):
        if session is not self.session:
            return
        if self.emit_session:
            self.emit_session(session['generation'], kind, value)
        else:
            self.emit(kind, value)

    def _request(self, session, path, payload=None, raw=None, headers=None):
        hdr = {'X-Sonia-Token': session['token'], **(headers or {})}
        data = raw
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode()
            hdr['Content-Type'] = 'application/json'
        req = request.Request(session['url'] + path, data=data, headers=hdr)
        try:
            with self.opener.open(req, timeout=120 if raw is not None else 15) as response:
                body = response.read()
                return json.loads(body) if 'application/json' in response.headers.get('Content-Type', '') else body
        except error.HTTPError as exc:
            try:
                detail = json.loads(exc.read()).get('error', str(exc))
            except (ValueError, AttributeError):
                detail = str(exc)
            raise RuntimeError(detail) from exc

    def connect(self, url):
        self.disconnect()
        session = {'url': normalize_url(url), 'token': '', 'generation': self.generation,
                   'stop': threading.Event(), 'muted': False}
        self.session = session
        threading.Thread(target=self._poll, args=(session,), daemon=True).start()

    def _hello(self, session, state):
        self.busy = state['busy']
        self._emit(session, 'event', {'type': 'hello', 'model': state['config']['model'],
                                    'history': state['messages'], 'state': 'thinking' if state['busy'] else 'listening'})
        approval = state.get('approval')
        if approval:
            self._event(session, {'kind': 'approval', 'value': approval})

    def _poll(self, session):
        try:
            parser = TokenParser()
            parser.feed(self._request(session, '/').decode('utf-8'))
            if not parser.token:
                raise RuntimeError('Questo indirizzo non espone Sonia Lite Web (token assente)')
            session['token'] = parser.token
            state = self._request(session, '/api/state')
            if session is not self.session:
                return
            self._hello(session, state)
            after = state['sequence']
            self.connected = True
            self._emit(session, 'connection', True)
            while not session['stop'].wait(.3):
                state = self._request(session, '/api/events?after=' + str(after))
                if session is not self.session:
                    return
                self.busy = state['busy']
                if state.get('reset_needed'):
                    state = self._request(session, '/api/state')
                    self._hello(session, state)
                else:
                    for event in state['events']:
                        self._event(session, event)
                after = state['sequence']
        except Exception as exc:
            self._emit(session, 'log', f'Errore di connessione: {exc}')
        finally:
            if session is self.session:
                self.connected = False
                self._emit(session, 'connection', False)

    def _event(self, session, event):
        kind, value = event['kind'], event['value']
        mapped = None
        if kind == 'user':
            mapped = {'type': 'user', 'text': value}
        elif kind == 'answer':
            mapped = {'type': 'assistant_done', 'text': value}
            if self.playback.enabled and not session['muted'] and value:
                self.tts_until = time.monotonic() + 15
                self._background(session, lambda: self._speak(session, value[:2000]))
        elif kind in ('phase', 'activity', 'error'):
            self._emit(session, 'log', ('Errore: ' if kind == 'error' else '') + str(value))
        elif kind == 'idle':
            mapped = {'type': 'state', 'state': 'listening'}
        elif kind == 'reset':
            mapped = {'type': 'chat_reset'}
        elif kind == 'model_changed':
            mapped = {'type': 'model_ready', 'model': value['model']}
        elif kind == 'approval' and value:
            mapped = {'type': 'command_request', 'id': value['id'], 'command': value['message']}
        elif kind == 'speech_audio' and self.playback.enabled and not session['muted']:
            self._background(session, lambda: self._audio(session, value['audio_id']))
        if mapped:
            self._emit(session, 'event', mapped)

    def _speak(self, session, text):
        try:
            self._request(session, '/api/tts/speak', {'text': text})
        except Exception:
            if session is self.session:
                self.tts_until = 0.0
            raise

    def _audio(self, session, ident):
        data = self._request(session, '/api/tts/audio?id=' + parse.quote(ident, safe=''))
        with wave.open(io.BytesIO(data)) as audio:
            if audio.getnchannels() != 1 or audio.getsampwidth() != 2:
                raise ValueError('Audio TTS non supportato: richiesto PCM16 mono')
            rate = audio.getframerate()
            pcm = audio.readframes(audio.getnframes())
        if not session['muted']:
            self._emit(session, 'wav', (rate, pcm))
        if session is self.session:
            self.tts_until = 0.0

    def _background(self, session, action):
        def run():
            if session is not self.session:
                return
            try:
                action()
            except Exception as exc:
                self._emit(session, 'log', f'Errore: {exc}')
        threading.Thread(target=run, daemon=True).start()

    def send(self, payload):
        if not self.connected:
            return False
        session = self.session
        kind = payload['type']
        if kind == 'text':
            self.busy = True
            session['muted'] = False
            path, body = '/api/chat', {'message': payload['text']}
        elif kind == 'approve':
            path, body = '/api/approve', {'id': payload['id'], 'approved': payload['ok']}
        elif kind == 'reset':
            path, body = '/api/reset', {}
        elif kind == 'stop':
            session['muted'] = True
            self.tts_until = 0.0
            self.playback.flush()
            return True
        else:
            return False
        self._background(session, lambda: self._request(session, path, body))
        return True

    def transcribe(self, pcm):
        if not self.connected or not pcm or self.transcribing:
            return False
        session = self.session
        self.transcribing = True
        def work():
            samples = array('h', pcm)
            if sys.byteorder != 'little':
                samples.byteswap()
            floats = array('f', (n / 32768 for n in samples))
            if sys.byteorder != 'little':
                floats.byteswap()
            result = self._request(session, '/api/stt/transcribe', raw=floats.tobytes(),
                                   headers={'Content-Type': 'application/octet-stream', 'X-Sonia-Sample-Rate': '16000'})
            text = result.get('text', '').strip()
            if text and session is self.session:
                self.send({'type': 'text', 'text': text})
            elif not text:
                self._emit(session, 'log', 'Trascrizione vuota: riprova a parlare.')
        def run():
            try:
                work()
            finally:
                if session is self.session:
                    self.transcribing = False
        self._background(session, run)
        return True

    def disconnect(self):
        self.generation += 1
        self.transcribing = False
        self.connected = False
        self.busy = False
        self.tts_until = 0.0
        if self.session:
            self.session['stop'].set()
        self.session = None
        self.microphone.stop()
        self.playback.flush()
