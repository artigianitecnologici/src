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
# import os
import os
import json
import re
from flask import Flask, request, jsonify, render_template
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import nltk
from nltk.corpus import stopwords
from nltk.tokenize import word_tokenize
from nltk.stem import WordNetLemmatizer
from spellchecker import SpellChecker
import random

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

# Inizializza il correttore ortografico
spell = SpellChecker(language='it')

def correct_spelling(text):
    """Corregge automaticamente gli errori di battitura nell'input dell'utente, escludendo parole chiave specifiche."""
    # Mantieni le parole chiave dell'argomento per evitare correzioni errate
    keywords = ["colosseo", "parlami", "dimmi", "info", "raccontami", "cos'è", "cose"]
    
    corrected_text = " ".join([
        word if word in keywords else spell.correction(word)
        for word in text.split()
    ])
    
    print(f"[DEBUG] Input corretto: {corrected_text}")
    return corrected_text

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
            return json.load(file).get("domande", [])
    except FileNotFoundError:
        return []

def save_knowledge(knowledge):
    """Salva la conoscenza in un file JSON."""
    with open(KNOWLEDGE_FILE, "w") as file:
        json.dump({"domande": knowledge}, file, indent=4)
    print("[DEBUG] Knowledge saved successfully")

# Modello TF-IDF
class ChatbotModel:
    def __init__(self):
        self.vectorizer = TfidfVectorizer()
        self.questions = []
        self.answers = []
        self.tfidf_matrix = None


    def update_model(self, knowledge, topic_filter=None):
        """Aggiorna il modello TF-IDF con la conoscenza attuale, filtrando per argomento."""
        filtered_knowledge = [
            item for item in knowledge if not topic_filter or item["argomento"] == topic_filter
        ]
        self.questions = [preprocess_text(item["pattern"]) for item in filtered_knowledge]
        
        # Gestisce sia "risposta" singola che "risposte" multiple
        self.answers = [
            item["risposta"] if "risposta" in item else 
            (random.choice(item["risposte"]) if "risposte" in item and item["risposte"] else "Risposta non disponibile")
            for item in filtered_knowledge
        ]
        
        if not self.questions:
            self.tfidf_matrix = None
            return
    
        self.tfidf_matrix = self.vectorizer.fit_transform(self.questions)


    def find_best_match(self, user_input, knowledge, topic_filter=None):
        """Trova la migliore corrispondenza per l'input dell'utente, filtrando per argomento."""
        corrected_input = correct_spelling(user_input.lower())
        print(f"[DEBUG] Input corretto: {corrected_input}")

        filtered_knowledge = [
            item for item in knowledge if not topic_filter or item["argomento"] == topic_filter
        ]
        
        for item in filtered_knowledge:
            try:
                if re.search(item["pattern"], corrected_input):
                    print(f"[DEBUG] Pattern trovato: {item['pattern']}")
                    # Restituisce una risposta casuale se la chiave è "risposte", altrimenti restituisce "risposta"
                    if "risposte" in item:
                        return random.choice(item["risposte"]), 1.0
                    elif "risposta" in item:
                        return item["risposta"], 1.0
            except re.error as e:
                print(f"[ERROR] Pattern non valido: {item['pattern']}, Errore: {e}")
            
        return None, None


chatbot_model = ChatbotModel()

@app.route("/")
def home():
    return render_template("index.html")

# @app.route("/get", methods=["GET"])
# def get_bot_response():
#     user_query = request.args.get('msg', '').strip()
#     topic_filter = request.args.get('topic', '').strip() or None
#     print(f"[DEBUG] Received message: {user_query}, Topic: {topic_filter}")

#     knowledge = load_knowledge()
#     print(f"[DEBUG] Loaded knowledge: {knowledge}")

#     chatbot_model.update_model(knowledge, topic_filter)
    
#     response, score = chatbot_model.find_best_match(user_query, knowledge, topic_filter)
#     print(f"[DEBUG] Best match response: {response}, Score: {score}")


#     if response:
#         return jsonify({"response": response, "action": "ok"})
    
#     if "martino" in user_query.lower():
#         return jsonify({"response": user_query, "action": "ok"})
    
#     return jsonify({"response": "Non ho capito. Puoi riformulare la domanda?", "action": "teach"})
@app.route("/get", methods=["GET"])
def get_bot_response():
    user_query = request.args.get('msg', '').strip()
    topic_filter = request.args.get('topic', '').strip() or None
    print(f"[DEBUG] Received message: {user_query}, Topic: {topic_filter}")

    # 🔒 Blocco risposta se l'input è vuoto
    if not user_query:
        return jsonify({"response": "", "action": "none"})

    knowledge = load_knowledge()
    print(f"[DEBUG] Loaded knowledge: {knowledge}")

    chatbot_model.update_model(knowledge, topic_filter)

    response, score = chatbot_model.find_best_match(user_query, knowledge, topic_filter)
    print(f"[DEBUG] Best match response: {response}, Score: {score}")

    if response:
        return jsonify({"response": response, "action": "ok"})

    if "martino" in user_query.lower():
        return jsonify({"response": user_query, "action": "ok"})

    #return jsonify({"response": "", "action": "none"})
    return jsonify({"response": "Non ho capito. Puoi riformulare la domanda?", "action": "teach"})


# 🌟 ROTTA PER L'INTERFACCIA DI ADDESTRAMENTO
@app.route("/training")
def training():
    return render_template("training.html")

# 🌟 ROTTA PER AGGIUNGERE NUOVE CONOSCENZE
@app.route("/teach", methods=["POST"])
def teach_bot():
    data = request.get_json()
    question = data.get("question", "").strip()
    answer = data.get("answer", "").strip()
    topic = data.get("topic", "").strip()

    if question and answer and topic:
        knowledge = load_knowledge()
        new_entry = {
            "pattern": question,
            "risposta": answer,
            "argomento": topic
        }
        knowledge.append(new_entry)
        save_knowledge(knowledge)
        chatbot_model.update_model(knowledge)
        return jsonify({"status": "success", "message": "Insegnamento salvato!"})
    
    return jsonify({"status": "error", "message": "Domanda, risposta o argomento mancante!"})

@app.route("/delete_knowledge", methods=["POST"])
def delete_knowledge():
    data = request.get_json()
    question = data.get("question", "").strip()

    if question:
        knowledge = load_knowledge()
        knowledge = [item for item in knowledge if item["pattern"] != question]
        save_knowledge(knowledge)
        chatbot_model.update_model(knowledge)
        return jsonify({"status": "success", "message": "Conoscenza eliminata!"})
    
    return jsonify({"status": "error", "message": "Domanda non trovata!"})


if __name__ == "__main__":
    os.makedirs(LOG_PATH, exist_ok=True)
    app.run(host="0.0.0.0", debug=True, port=5500)
