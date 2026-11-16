"""Test senza hardware del filtro di attivazione e del listener locale."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

from wake_word import WakeGate, WakeListener, load_wake_word, confirmed_wake
from sonia_client import SoniaUI, AudioPlayback


def final_result(text, confidence=1.0):
    return {'text': text, 'result': [
        {'word': word, 'conf': confidence, 'start': i * .3, 'end': (i + 1) * .3}
        for i, word in enumerate(text.split())]}


class ConfirmationTests(unittest.TestCase):
    def test_accepts_only_explicit_final_wake_word(self):
        self.assertTrue(confirmed_wake(final_result('sonia mi dici gli appuntamenti'), 'sonia'))
        self.assertTrue(confirmed_wake(final_result('ciao aurora'), 'aurora'))
        self.assertTrue(confirmed_wake(final_result('ciao robot'), 'ciao robot'))
        self.assertFalse(confirmed_wake(final_result('oggi sonia arriva'), 'sonia'))
        self.assertFalse(confirmed_wake(final_result('bisogna andare'), 'sonia'))
        self.assertFalse(confirmed_wake(final_result('sonia', .4), 'sonia'))
        self.assertFalse(confirmed_wake({'partial': 'sonia'}, 'sonia'))
        self.assertFalse(confirmed_wake({'text': 'sonia'}, 'sonia'))



class ConfigTests(unittest.TestCase):
    def test_custom_word_and_reload(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'config.json'
            path.write_text('{"wake_word": "  Aurora  "}')
            word = load_wake_word(path)
            self.assertEqual(word, 'aurora')
            gate = WakeGate(word)
            self.assertEqual(gate.accept('Aurora, che tempo fa?', 1), 'che tempo fa?')
            self.assertIsNone(gate.accept('Sonia che tempo fa?', 2))
            path.write_text('{"wake_word": "ciao robot"}')
            self.assertEqual(load_wake_word(path), 'ciao robot')

    def test_invalid_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'config.json'
            for config in ('{', '[]', '{}', '{"wake_word": null}',
                           '{"wake_word": ""}', '{"wake_word": "[unk]"}'):
                path.write_text(config)
                with self.subTest(config=config), self.assertRaises(ValueError):
                    load_wake_word(path)
            with self.assertRaises(ValueError):
                load_wake_word(Path(tmp) / 'missing.json')


class GateTests(unittest.TestCase):
    def test_same_sentence(self):
        gate = WakeGate()
        self.assertEqual(gate.accept('Sonia, che tempo fa ad Ariccia oggi?', 1),
                         'che tempo fa ad Ariccia oggi?')
        self.assertEqual(gate.accept('ehi sonia accendi la luce', 2), 'accendi la luce')

    def test_non_wake_speech_is_ignored(self):
        gate = WakeGate()
        for text in ('che tempo fa oggi', 'bisogna andare', 'sonietta ciao', 'ho visto sonia ieri'):
            self.assertIsNone(gate.accept(text, 1))

    def test_wake_then_question_with_timeout(self):
        gate = WakeGate()
        self.assertIsNone(gate.accept('sonia', 1))
        self.assertEqual(gate.accept('che tempo fa', 3), 'che tempo fa')
        self.assertIsNone(gate.accept('altra frase', 4))
        gate.accept('sonia', 5)
        self.assertIsNone(gate.accept('troppo tardi', 16))
        self.assertEqual(gate.until, 0)

    def test_pause_clears_activation(self):
        gate = WakeGate()
        gate.accept('sonia', 1)
        gate.reset()
        self.assertIsNone(gate.accept('non inviare', 2))


class ListenerTests(unittest.TestCase):
    def test_listener_sends_only_activated_final_phrase(self):
        events = []
        ready = threading.Event()
        waiting = threading.Event()
        streams = []
        class Recognizer:
            def SetWords(self, enabled):
                assert enabled
            def AcceptWaveform(self, data):
                self.data = data
                return True
            def Result(self):
                return json.dumps(final_result(self.data.decode()))
            def Reset(self):
                pass
        class Stream:
            def __init__(self, **kwargs):
                self.callback = kwargs['callback']
                streams.append(self)
            def __enter__(self):
                self.callback(b'conversazione privata', 1600, None, None)
                self.callback(b'sonia che tempo fa ad ariccia', 1600, None, None)
                for _ in range(14):
                    self.callback(bytes(3200), 1600, None, None)
                return self
            def __exit__(self, *args):
                pass
        def emit(kind, value):
            events.append((kind, value))
            if kind == 'wake_audio':
                ready.set()
            if kind == 'wake_status' and str(value).startswith('In attesa') and ready.is_set():
                waiting.set()
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)
            (model / 'am').mkdir()
            (model / 'am/final.mdl').touch()
            vosk = types.SimpleNamespace(Model=Mock(), KaldiRecognizer=lambda *args: Recognizer())
            sd = types.SimpleNamespace(RawInputStream=Stream)
            with patch.dict(sys.modules, {'vosk': vosk, 'sounddevice': sd}):
                listener = WakeListener(emit, lambda: True, model_path=model)
                listener.start()
                try:
                    self.assertTrue(ready.wait(2))
                    self.assertTrue(waiting.wait(2), events)
                    ready.clear()
                    streams[0].callback(b'che tempo fa oggi', 1600, None, None)
                    for _ in range(14):
                        streams[0].callback(bytes(3200), 1600, None, None)
                    self.assertFalse(ready.wait(.3), 'Seconda richiesta inviata senza wake word')
                finally:
                    listener.stop()
        commands = [value[1] for kind, value in events if kind == 'wake_audio']
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith(b'sonia che tempo fa ad ariccia'))
        self.assertFalse(listener.active)
        self.assertFalse(listener.thread.is_alive())

    def test_wake_only_segment_keeps_beginning_of_question(self):
        events = []
        ready = threading.Event()
        first = b'sonia mi dici gli '
        second = b'appuntamenti di oggi'
        class Recognizer:
            def SetWords(self, enabled):
                assert enabled
            def __init__(self):
                self.count = 0
            def AcceptWaveform(self, data):
                self.count += 1
                return True
            def Result(self):
                # Il decoder riconosce solo il nome nel primo segmento,
                # sebbene l'audio contenga già parole della domanda.
                return json.dumps(final_result('sonia' if self.count == 1 else '[unk]'))
            def Reset(self):
                pass
        class Stream:
            def __init__(self, **kwargs):
                self.callback = kwargs['callback']
            def __enter__(self):
                self.callback(first, 1600, None, None)
                self.callback(second, 1600, None, None)
                for _ in range(14):
                    self.callback(bytes(3200), 1600, None, None)
                return self
            def __exit__(self, *args):
                pass
        def emit(kind, value):
            events.append((kind, value))
            if kind == 'wake_audio':
                ready.set()
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)
            (model / 'am').mkdir()
            (model / 'am/final.mdl').touch()
            vosk = types.SimpleNamespace(Model=Mock(), KaldiRecognizer=lambda *args: Recognizer())
            with patch.dict(sys.modules, {'vosk': vosk, 'sounddevice': types.SimpleNamespace(RawInputStream=Stream)}):
                listener = WakeListener(emit, lambda: True, model_path=model)
                listener.start()
                try:
                    self.assertTrue(ready.wait(2))
                finally:
                    listener.stop()
        recordings = [value[1] for kind, value in events if kind == 'wake_audio']
        self.assertEqual(len(recordings), 1)
        self.assertTrue(recordings[0].startswith(first + second))

    def test_final_wake_starts_transcription_without_digital_silence(self):
        events = []
        ready = threading.Event()
        voice = b'\x00\x10' * 1600
        class Recognizer:
            def SetWords(self, enabled):
                assert enabled
            def __init__(self):
                self.count = 0
            def AcceptWaveform(self, data):
                self.count += 1
                return self.count == 2
            def Result(self):
                return json.dumps(final_result('sonia'))
            def PartialResult(self):
                return '{"partial": ""}'
            def Reset(self):
                pass
        class Stream:
            def __init__(self, **kwargs):
                self.callback = kwargs['callback']
            def __enter__(self):
                for _ in range(14):
                    self.callback(voice, 1600, None, None)
                return self
            def __exit__(self, *args):
                pass
        def emit(kind, value):
            events.append((kind, value))
            if kind == 'wake_audio':
                ready.set()
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)
            (model / 'am').mkdir()
            (model / 'am/final.mdl').touch()
            vosk = types.SimpleNamespace(Model=Mock(), KaldiRecognizer=lambda *args: Recognizer())
            with patch.dict(sys.modules, {'vosk': vosk, 'sounddevice': types.SimpleNamespace(RawInputStream=Stream)}):
                listener = WakeListener(emit, lambda: True, model_path=model)
                listener.start()
                try:
                    self.assertTrue(ready.wait(2), events)
                finally:
                    listener.stop()
        recordings = [v[1] for k, v in events if k == 'wake_audio']
        self.assertEqual(len(recordings), 1)
        self.assertTrue(recordings[0].startswith(voice * 2))

    def test_false_partial_wake_cannot_send_audio(self):
        events = []
        processed = threading.Event()
        class Recognizer:
            def __init__(self):
                self.count = 0
            def SetWords(self, enabled):
                pass
            def AcceptWaveform(self, data):
                self.count += 1
                return self.count == 20
            def PartialResult(self):
                raise AssertionError('Un parziale non deve autorizzare la trascrizione')
            def Result(self):
                processed.set()
                return json.dumps(final_result('che tempo fa oggi'))
            def Reset(self):
                pass
        class Stream:
            def __init__(self, **kwargs):
                self.callback = kwargs['callback']
            def __enter__(self):
                for _ in range(20):
                    self.callback(bytes(3200), 1600, None, None)
                return self
            def __exit__(self, *args):
                pass
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)
            (model / 'am').mkdir()
            (model / 'am/final.mdl').touch()
            factory = Mock(side_effect=lambda model, rate: Recognizer())
            vosk = types.SimpleNamespace(Model=Mock(), KaldiRecognizer=factory)
            with patch.dict(sys.modules, {'vosk': vosk, 'sounddevice': types.SimpleNamespace(RawInputStream=Stream)}):
                listener = WakeListener(lambda *event: events.append(event), lambda: True, model_path=model)
                listener.start()
                try:
                    self.assertTrue(processed.wait(2), events)
                finally:
                    listener.stop()
            self.assertEqual(len(factory.call_args.args), 2, 'Non usare la grammatica forzata')
        self.assertFalse(any(k == 'wake_audio' for k, _ in events), events)

    def test_pause_reason_updates_and_listener_resumes(self):
        first = threading.Event()
        changed = threading.Event()
        resumed = threading.Event()
        state = {'reason': 'trascrizione in corso sul server'}
        events = []
        def emit(kind, value):
            events.append((kind, value))
            if kind != 'wake_status':
                return
            if value == 'Ascolto sospeso · trascrizione in corso sul server':
                first.set()
            elif value == 'Ascolto sospeso · riproduzione della risposta vocale':
                changed.set()
            elif str(value).startswith('In attesa') and changed.is_set():
                resumed.set()
        class Stream:
            def __init__(self, **kwargs):
                pass
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)
            (model / 'am').mkdir()
            (model / 'am/final.mdl').touch()
            vosk = types.SimpleNamespace(Model=Mock(), KaldiRecognizer=Mock())
            with patch.dict(sys.modules, {'vosk': vosk, 'sounddevice': types.SimpleNamespace(RawInputStream=Stream)}):
                listener = WakeListener(emit, lambda: False, model_path=model,
                                        pause_reason=lambda: state['reason'])
                listener.start()
                try:
                    self.assertTrue(first.wait(2), events)
                    state['reason'] = 'riproduzione della risposta vocale'
                    self.assertTrue(changed.wait(2), events)
                    state['reason'] = ''
                    self.assertTrue(resumed.wait(2), events)
                finally:
                    listener.stop()
        self.assertFalse(any(k == 'wake_audio' for k, _ in events))

    def test_model_missing_reports_setup_and_stops(self):
        events = []
        with tempfile.TemporaryDirectory() as tmp:
            fake = types.SimpleNamespace(Model=Mock(), KaldiRecognizer=Mock())
            with patch.dict(sys.modules, {'vosk': fake, 'sounddevice': Mock()}):
                listener = WakeListener(lambda *event: events.append(event), lambda: True, model_path=tmp)
                listener.start()
                listener.thread.join(timeout=2)
        self.assertFalse(listener.active)
        self.assertTrue(any(k == 'log' and 'installa_wake_word.sh' in v for k, v in events))

    def test_listening_pauses_for_server_and_own_voice(self):
        ui = SoniaUI.__new__(SoniaUI)
        ui.connection = types.SimpleNamespace(connected=True, busy=False, tts_until=0, transcribing=False)
        ui.microphone = types.SimpleNamespace(active=False)
        ui.playback = AudioPlayback(Mock())
        ui.playback.stream = types.SimpleNamespace(active=True)
        ui.wake_hold_until = 0
        self.assertTrue(ui._wake_available())
        ui.connection.busy = True
        self.assertFalse(ui._wake_available())
        ui.connection.busy = False
        ui.playback.buffer = bytearray(b'playing')
        ui.playback.last_data = time.monotonic()
        self.assertFalse(ui._wake_available())
        ui.playback.buffer.clear()
        ui.connection.tts_until = float('inf')
        self.assertFalse(ui._wake_available())
        ui.connection.tts_until = 0
        ui.connection.connected = False
        self.assertFalse(ui._wake_available())


class PlaybackRecoveryTests(unittest.TestCase):
    def playback(self, active=True):
        errors = []
        playback = AudioPlayback(errors.append)
        playback.stream = types.SimpleNamespace(active=active)
        playback.buffer.extend(b'\x01\x00' * 1000)
        playback.last_data = 100
        return playback, errors

    def test_stopped_output_cannot_keep_wake_paused(self):
        playback, errors = self.playback(active=False)
        self.assertFalse(playback.is_busy(now=100.1))
        self.assertFalse(playback.buffer)
        self.assertEqual(len(errors), 1)
        self.assertFalse(playback.is_busy(now=101))
        self.assertEqual(len(errors), 1)

    def test_missing_callbacks_clear_abandoned_audio(self):
        playback, errors = self.playback()
        self.assertFalse(playback.is_busy(now=103))
        self.assertFalse(playback.buffer)
        self.assertEqual(len(errors), 1)

    def test_long_audio_with_recent_output_remains_paused(self):
        playback, errors = self.playback()
        playback.last_output = 200
        self.assertTrue(playback.is_busy(now=200.1))
        self.assertTrue(playback.buffer)
        self.assertFalse(errors)


if __name__ == '__main__':
    unittest.main()
