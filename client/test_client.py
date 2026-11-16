"""Integrazione del client con le API reali di Sonia Lite Web, modello simulato."""
from array import array
import io
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
import wave

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sonia_lite
import sonia_web
from sonia_http import SoniaConnection, normalize_url
from sonia_client import AudioPlayback, Microphone, MIC_FRAMES, SoniaUI


class URLTests(unittest.TestCase):
    def test_lan_url(self):
        self.assertEqual(normalize_url('http://192.168.1.199:8096/'), 'http://192.168.1.199:8096')
        self.assertEqual(normalize_url('https://sonia.example'), 'https://sonia.example')

    def test_reject_invalid_urls(self):
        for url in ('ftp://localhost', 'ws://localhost/ws', 'http://user:pass@host',
                    'http://0.0.0.0:8096', 'http://host/api/state', 'http://host:bad', 'http://host?q=1'):
            with self.subTest(url=url), self.assertRaises(ValueError):
                normalize_url(url)


class ClientTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        cfg = {**sonia_lite.DEFAULTS, 'model': 'modello-test', 'skills_dir': tmp.name}
        self.state = sonia_web.AppState(cfg)
        self.state.agent.ask = Mock(return_value='Ciao dal server Sonia!')
        self.web = sonia_web.SoniaHTTP(('127.0.0.1', 0), self.state)
        self.web.speech = Mock()
        self.web.speech.transcribe.return_value = 'Messaggio dettato'
        threading.Thread(target=self.web.serve_forever, daemon=True).start()
        self.addCleanup(self.web.server_close)
        self.addCleanup(self.web.shutdown)
        self.events = []
        self.playback = Mock(enabled=False)
        self.conn = SoniaConnection(lambda kind, value: self.events.append((kind, value)), self.playback, Mock())
        self.addCleanup(self.conn.disconnect)
        self.url = f'http://127.0.0.1:{self.web.server_port}'

    def wait_for(self, condition):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.02)
        self.fail(f'Attesa scaduta: {self.events}')

    def connect(self):
        self.conn.connect(self.url)
        self.wait_for(lambda: self.conn.connected)

    def has_event(self, kind):
        return any(k == 'event' and v['type'] == kind for k, v in self.events)

    def test_connect_and_chat_using_real_server_routes(self):
        self.connect()
        self.assertTrue(self.has_event('hello'))
        self.assertTrue(self.conn.send({'type': 'text', 'text': 'Ciao'}))
        self.wait_for(lambda: self.has_event('assistant_done'))
        self.state.agent.ask.assert_called_once_with('Ciao')
        answers = [v['text'] for k, v in self.events if k == 'event' and v['type'] == 'assistant_done']
        self.assertEqual(answers, ['Ciao dal server Sonia!'])

    def test_approval_and_reset(self):
        self.connect()
        result = []
        threading.Thread(target=lambda: result.append(self.state.request_approval('Esegui skill?')), daemon=True).start()
        self.wait_for(lambda: self.has_event('command_request'))
        approval = next(v for k, v in self.events if k == 'event' and v['type'] == 'command_request')
        self.conn.send({'type': 'approve', 'id': approval['id'], 'ok': True})
        self.wait_for(lambda: bool(result))
        self.assertEqual(result, [True])
        self.conn.send({'type': 'reset'})
        self.wait_for(lambda: self.has_event('chat_reset') or
                      sum(k == 'event' and v['type'] == 'hello' for k, v in self.events) > 1)
        self.assertEqual(self.state.messages(), [])

    def test_microphone_pcm16_converted_to_float32_and_sent_to_chat(self):
        self.connect()
        self.conn.transcribe(b'\x00\x40\x00\xc0')
        self.wait_for(lambda: self.has_event('assistant_done'))
        raw, rate = self.web.speech.transcribe.call_args.args
        samples = array('f')
        samples.frombytes(raw)
        if sys.byteorder != 'little':
            samples.byteswap()
        self.assertEqual(list(samples), [.5, -.5])
        self.assertEqual(rate, 16000)
        self.state.agent.ask.assert_called_once_with('Messaggio dettato')

    def test_tts_download_and_stop_audio(self):
        self.connect()
        self.playback.enabled = True
        buf = io.BytesIO()
        with wave.open(buf, 'wb') as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(24000)
            wav.writeframes(b'\x01\x00' * 100)
        self.web.speech.get.return_value = buf.getvalue()
        self.state.emit('speech_audio', {'audio_id': 'abc'})
        self.wait_for(lambda: any(k == 'wav' for k, _ in self.events))
        self.assertIn(('wav', (24000, b'\x01\x00' * 100)), self.events)
        self.conn.tts_until = float('inf')
        self.conn.send({'type': 'stop'})
        self.assertEqual(self.conn.tts_until, 0)
        self.assertTrue(self.conn.session['muted'])
        self.playback.flush.assert_called()

    def test_reconnect_replaces_session(self):
        self.connect()
        old = self.conn.session
        self.conn.connect(self.url)
        self.wait_for(lambda: self.conn.connected)
        self.assertTrue(old['stop'].is_set())
        count = len(self.events)
        self.conn._emit(old, 'log', 'vecchio evento')
        self.assertEqual(len(self.events), count)

    def test_server_rejection_reaches_ui(self):
        self.connect()
        self.state.busy = True
        self.conn.send({'type': 'text', 'text': 'Ciao'})
        self.wait_for(lambda: any(k == 'log' and 'già elaborando' in v for k, v in self.events))


class PlaybackTests(unittest.TestCase):
    def test_microphone_preserves_recording_until_finish(self):
        mic = Microphone(Mock())
        mic.active = True
        packet = b'\x01\x00' * MIC_FRAMES
        for _ in range(20):
            mic._callback(packet, MIC_FRAMES, None, None)
        self.assertEqual(mic.finish(), packet * 20)
        self.assertFalse(mic.active)
        self.assertEqual(mic.finish(), b'')

    def test_ui_hello_with_empty_history(self):
        ui = SoniaUI.__new__(SoniaUI)
        ui.model_label, ui.audio_label, ui.chat = Mock(), Mock(), Mock()
        ui._system, ui._state = Mock(), Mock()
        ui._event({'type': 'hello', 'model': 'test', 'history': []})
        ui._system.assert_called_once()
        ui._state.assert_called_once_with('listening')

    def test_long_wav_is_not_truncated_to_five_seconds(self):
        playback = AudioPlayback(Mock())
        playback.stream = Mock()
        playback.rate = 16000
        data = b'\x01\x00' * (16000 * 8)
        playback.add(data)
        self.assertEqual(bytes(playback.buffer), data)

    def test_ui_discards_old_session_events(self):
        ui = SoniaUI.__new__(SoniaUI)
        ui.events = queue.Queue()
        ui.connection = Mock(generation=4)
        ui.root, ui.microphone, ui.playback, ui.status = Mock(), Mock(), Mock(), Mock()
        ui._event = Mock()
        ui.events.put((2, 'event', {'type': 'assistant_done', 'text': 'vecchio'}))
        ui.events.put((4, 'event', {'type': 'assistant_done', 'text': 'nuovo'}))
        ui._poll()
        ui._event.assert_called_once_with({'type': 'assistant_done', 'text': 'nuovo'})


if __name__ == '__main__':
    unittest.main()
