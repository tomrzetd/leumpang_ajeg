// Suara: pengenalan ucapan (SpeechRecognizer, id-ID, utamakan offline) + sintesis ucapan (TextToSpeech id-ID).
// Ucapan dikenali -> speech/heard + behavior/request. speech/say -> diucapkan. Mendengar dijeda saat robot berbicara.
package id.leumpang.mata

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import id.leumpang.bus.Bus
import id.leumpang.bus.J
import id.leumpang.bus.Module
import id.leumpang.bus.ModuleInfo
import id.leumpang.bus.Subscription
import id.leumpang.perception.Speech
import java.util.Locale

class SpeechModule(private val ctx: Context, private val settings: Settings, private val log: (String) -> Unit) : Module {
    override val info = ModuleInfo(
        "speech", "Pengenalan & sintesis ucapan Bahasa Indonesia",
        listOf("speech/heard", "behavior/request"), listOf("speech/say"),
    )
    private val main = Handler(Looper.getMainLooper())
    private var bus: Bus? = null
    private var tts: TextToSpeech? = null
    private var ttsReady = false
    @Volatile private var speaking = false
    private var recognizer: SpeechRecognizer? = null
    private var listening = false
    private var sub: Subscription? = null
    @Volatile var lastHeard = ""; private set
    @Volatile var status = "mati"; private set

    override fun start(bus: Bus) {
        this.bus = bus
        main.post {
            tts = TextToSpeech(ctx) { st ->
                ttsReady = st == TextToSpeech.SUCCESS
                if (ttsReady) {
                    val r = tts?.setLanguage(Locale("id", "ID"))
                    if (r == TextToSpeech.LANG_MISSING_DATA || r == TextToSpeech.LANG_NOT_SUPPORTED) log("TTS: suara Bahasa Indonesia belum terpasang (Pengaturan > Text-to-speech)")
                    tts?.setOnUtteranceProgressListener(object : UtteranceProgressListener() {
                        override fun onStart(id: String?) { speaking = true }
                        override fun onDone(id: String?) { speaking = false; main.postDelayed({ if (settings.speech) listen() }, 400) }
                        @Deprecated("deprecated") override fun onError(id: String?) { speaking = false }
                    })
                }
            }
            if (settings.speech) enableListening(true)
        }
        sub = bus.subscribe("speech/say", "speech") { m -> J.str(m.data, "text")?.let { say(it) } }
    }

    fun say(text: String) {
        log("robot: $text")
        if (!settings.tts || !ttsReady) return
        main.post {
            stopListening()
            tts?.speak(text, TextToSpeech.QUEUE_ADD, null, "u${System.nanoTime()}")
        }
    }

    fun enableListening(on: Boolean) = main.post {
        if (on && !SpeechRecognizer.isRecognitionAvailable(ctx)) { status = "pengenalan ucapan tidak tersedia di HP ini"; log(status); return@post }
        if (on) listen() else { stopListening(); recognizer?.destroy(); recognizer = null; status = "mati" }
    }

    private fun stopListening() { if (listening) { recognizer?.cancel(); listening = false } }

    private fun listen() {
        if (!settings.speech || speaking || listening) return
        val r = recognizer ?: SpeechRecognizer.createSpeechRecognizer(ctx).also { recognizer = it; it.setRecognitionListener(listener) }
        val i = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
            putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
            putExtra(RecognizerIntent.EXTRA_LANGUAGE, "id-ID")
            putExtra(RecognizerIntent.EXTRA_PREFER_OFFLINE, true)
            putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 3)
        }
        listening = true
        status = if (settings.requireWake) "mendengar (awali dengan \"robot\" / \"leumpang\")" else "mendengar"
        r.startListening(i)
    }

    private val listener = object : RecognitionListener {
        override fun onResults(b: Bundle?) {
            listening = false
            val all = b?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION).orEmpty()
            val text = all.firstOrNull().orEmpty()
            if (text.isNotBlank()) {
                lastHeard = text
                bus?.publish("speech/heard", J.obj("text" to text, "alternatives" to all), "speech", retain = false)
                val req = all.firstNotNullOfOrNull { Speech.parse(it, settings.requireWake) }
                if (req != null) bus?.publish("behavior/request", RequestJson.encode(req), "speech", retain = false)
            }
            main.postDelayed({ listen() }, 250)
        }
        override fun onError(error: Int) {
            listening = false
            // NO_MATCH / SPEECH_TIMEOUT normal saat sepi; ulangi dengan jeda
            main.postDelayed({ listen() }, if (error == SpeechRecognizer.ERROR_RECOGNIZER_BUSY) 1500 else 400)
        }
        override fun onReadyForSpeech(p: Bundle?) {}
        override fun onBeginningOfSpeech() {}
        override fun onRmsChanged(v: Float) {}
        override fun onBufferReceived(b: ByteArray?) {}
        override fun onEndOfSpeech() {}
        override fun onPartialResults(b: Bundle?) {}
        override fun onEvent(t: Int, b: Bundle?) {}
    }

    override fun stop() {
        sub?.cancel()
        main.post { recognizer?.destroy(); recognizer = null; tts?.shutdown(); tts = null }
    }
}
