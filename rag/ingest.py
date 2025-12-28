import os
from qdrant_client import QdrantClient
from sentence_transformers import SentenceTransformer

# 🔒 HARD-CODED PATH (FINAL FIX)
DOCS_PATH = r"C:\Users\Hafiz Rashid\OneDrive\Desktop\Hackathon\GIAIC\docs\tutorial-basics"


model = SentenceTransformer("all-MiniLM-L6-v2")

client = QdrantClient(
    url="https://6bef06ec-6110-4599-9066-4872dde17237.europe-west3-0.gcp.cloud.qdrant.io:6333", 
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.O-yuuycAhIdMXTQ2DGDnBPOrknD6yOH4pwd0OlJjMYU",
    )

client.recreate_collection(
    collection_name="book",
    vectors_config={"size": 384, "distance": "Cosine"},
)

for f in os.listdir(r"C:\Users\Hafiz Rashid\OneDrive\Desktop\Hackathon\GIAIC\docs\tutorial-basics"):
    if f.endswith(".md"):
        with open(os.path.join(DOCS_PATH, f), "r", encoding="utf-8") as file:
            text = file.read()
            vector = model.encode(text).tolist()

            client.upsert(
                collection_name="book",
                points=[
                    {
                        "id": hash(f),
                        "vector": vector,
                        "payload": {"text": text},
                    }
                ],
            )

print("✅ INGEST COMPLETED")
