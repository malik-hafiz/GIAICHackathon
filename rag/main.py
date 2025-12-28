from fastapi import FastAPI
from qdrant_client import QdrantClient
from sentence_transformers import SentenceTransformer
import subprocess

app = FastAPI()

model = SentenceTransformer("all-MiniLM-L6-v2")
client = QdrantClient(url="https://6bef06ec-6110-4599-9066-4872dde17237.europe-west3-0.gcp.cloud.qdrant.io:6333", api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.O-yuuycAhIdMXTQ2DGDnBPOrknD6yOH4pwd0OlJjMYU")

@app.post("/ask")
def ask(q: str):
    vec = model.encode(q).tolist()
    res = client.search("book", vec, limit=2)

    context = "\n".join([r.payload["text"] for r in res])

    prompt = f"""
Answer ONLY from the context.
If not found say: Not in book.

Context:
{context}

Question:
{q}
"""

    answer = subprocess.check_output(
        ["qwen", "chat", prompt],
        text=True
    )
    return {"answer": answer}
