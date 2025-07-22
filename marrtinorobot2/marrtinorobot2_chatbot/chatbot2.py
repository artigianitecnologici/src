# Copyright 2025 robotics-3d.com 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com

#!/usr/bin/python3
import os
import json
from flask import Flask, request, jsonify, render_template
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import nltk
from nltk.corpus import stopwords
from nltk.tokenize import word_tokenize
from nltk.stem import WordNetLemmatizer

# Scarica risorse di NLTK
nltk.download('punkt')
nltk.download('stopwords')
nltk.download('wordnet')

# Percorsi
PATH = os.path.expandvars("$HOME/src/marrtinorobot2/marrtinorobot2_chatbot/")
LOG_PATH = os.path.join(PATH, "log")
KNOWLEDGE_FILE = os.path.join(PATH, "knowledge.json")

# Inizializzazione
app = Flask(__name__)
app.static_folder = 'static'

# Preprocessing del testo con NLTK
lemmatizer = WordNetLemmatizer()
stop_words = set(stopwords.words('italian'))

def preprocess_text(text):
    """Pulisce e normalizza il testo."""
    tokens = word_tokenize(text.lower())
    tokens = [lemmatizer.lemmatize(word) for word in tokens if word.isalnum() and word not in stop_words]
    return " ".join(tokens) if tokens else "no_content"

# Gestione della conoscenza
def load_knowledge():
    """Carica la conoscenza da un file JSON."""
    try:
        with open(KNOWLEDGE_FILE, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        return {}

def save_knowledge(knowledge):
    """Salva la conoscenza in un file JSON."""
    with open(KNOWLEDGE_FILE, "w") as file:
        json.dump(knowledge, file, indent=4)
    print("[DEBUG] Knowledge saved successfully")

# Modello TF-IDF
class ChatbotModel:
    def __init__(self):
        self.vectorizer = TfidfVectorizer()
        self.questions = []
        self.answers = []
        self.tfidf_matrix = None

    def update_model(self, knowledge):
        """Aggiorna il modello TF-IDF con la conoscenza attuale."""
        self.questions = [preprocess_text(q) for q in knowledge.keys()]
        self.answers = list(knowledge.values())
        if not self.questions:
            self.tfidf_matrix = None
            return
        self.tfidf_matrix = self.vectorizer.fit_transform(self.questions)

    def find_best_match(self, user_input):
        """Trova la migliore corrispondenza per l'input dell'utente."""
        if self.tfidf_matrix is None or self.tfidf_matrix.shape[0] == 0 or not self.questions:
        #if not self.tfidf_matrix or not self.questions:
            return None, None
        
        input_vector = self.vectorizer.transform([preprocess_text(user_input)])
        scores = cosine_similarity(input_vector, self.tfidf_matrix)
        
        best_index = scores.argmax()
        best_score = scores[0, best_index]
        
        print(f"[DEBUG] Best score: {best_score}, Best index: {best_index}")
        
        if float(best_score) > 0.5:  # Soglia di similarità
            return self.answers[best_index], best_score
        
        return None, None

chatbot_model = ChatbotModel()

# Logging
def log_to_file(question, answer):
    """Registra le interazioni nel file di log."""
    log_file = os.path.join(LOG_PATH, "log.txt")
    with open(log_file, "a") as file:
        file.write(f"QUESTION: {question}; ANSWER: {answer}\n")

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/get", methods=["GET"])
def get_bot_response():
    user_query = request.args.get('msg', '').strip()
    print(f"[DEBUG] Received message: {user_query}")

    knowledge = load_knowledge()
    print(f"[DEBUG] Loaded knowledge: {knowledge}")

    # Aggiorna il modello
    chatbot_model.update_model(knowledge)
    
    # Controlla se il modello è vuoto
    if chatbot_model.tfidf_matrix is None:
        bot_response = "Non conosco ancora nulla. Vuoi insegnarmi qualcosa? Scrivi: 'Insegna: domanda | risposta'"
        log_to_file(user_query, bot_response)
        return jsonify({"response": bot_response, "action": "teach"})

    # Cerca la risposta migliore
    #try:
    response, score = chatbot_model.find_best_match(user_query)
    print(f"[DEBUG] Best match response: {response}, Score: {score}")

    if response:
        log_to_file(user_query, response)
        return jsonify({"response": response, "action": "ok"})
    
    default_responses = [
        "Non sono sicuro di cosa dire. Vuoi insegnarmi qualcosa?",
        "Hmm, questa domanda mi mette in difficoltà! Mi insegni la risposta?",
        "Non so rispondere a questo... ma posso imparare da te!"
    ]

    from random import choice
    bot_response = choice(default_responses)
    
    log_to_file(user_query, bot_response)
    return jsonify({"response": bot_response, "action": "teach"})

    #except Exception as e:
    #    print(f"[ERROR] Exception occurred: {e}")
    #    return jsonify({"response": "Si è verificato un errore. Riprova più tardi.", "action": "error"})


@app.route("/teach", methods=["POST"])
def teach_bot():
    print("[DEBUG] Teach endpoint called")
    data = request.get_json()
    print(f"[DEBUG] Received data: {data}")
    question = data.get("question", "").strip()
    answer = data.get("answer", "").strip()
     
    if question and answer:
        knowledge = load_knowledge()
        knowledge[question] = answer
        print(f"[DEBUG] Adding knowledge - Question: {question}, Answer: {answer}")
        save_knowledge(knowledge)
        chatbot_model.update_model(knowledge)  # Aggiorna il modello TF-IDF
        log_to_file(question, answer)
        return jsonify({"status": "success", "message": "Insegnamento salvato!"})
    return jsonify({"status": "error", "message": "Domanda o risposta mancante!"})

@app.route("/teach_interactive", methods=["GET"])
def teach_interactive():
    print("[DEBUG] Entered teach_interactive")
    user_query = request.args.get('msg', '').strip()
    print(f"[DEBUG] Received teach command: {user_query}")

    if user_query.lower().startswith("insegna:"):
        print("[DEBUG] Teach command detected")
        try:
            _, content = user_query.split(":", 1)
            print(f"[DEBUG] Command content: {content}")
            question, answer = map(str.strip, content.split("|", 1))
            print(f"[DEBUG] Parsed - Question: {question}, Answer: {answer}")
            knowledge = load_knowledge()
            knowledge[question] = answer
            save_knowledge(knowledge)
            chatbot_model.update_model(knowledge)
            log_to_file(question, answer)
            return jsonify({"response": "Grazie! Ho imparato qualcosa di nuovo!", "action": "ok"})
        except ValueError:
            print("[DEBUG] Invalid format in teach command")
            return jsonify({"response": "Formato non valido. Usa: 'Insegna: domanda | risposta'", "action": "error"})
    print("[DEBUG] No valid teach command detected")
    return jsonify({"response": "Non ho capito. Usa: 'Insegna: domanda | risposta'", "action": "error"})

# Avvia il server Flask
if __name__ == "__main__":
    os.makedirs(LOG_PATH, exist_ok=True)  # Crea la directory di log se non esiste
    app.run(host="0.0.0.0", debug=True, port=5500)
