# from fastapi import FastAPI
# from pydantic import BaseModel
# from qdrant_client import QdrantClient
# from sentence_transformers import SentenceTransformer

# app = FastAPI()

# model = SentenceTransformer("all-MiniLM-L6-v2")

# client = QdrantClient(
#     url="https://6bef06ec-6110-4599-9066-4872dde17237.europe-west3-0.gcp.cloud.qdrant.io:6333",
#     api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.O-yuuycAhIdMXTQ2DGDnBPOrknD6yOH4pwd0OlJjMYU",
# )

# class Query(BaseModel):
#     question: str

# @app.post("/ask")
# def ask(query: Query):
#     vector = model.encode(query.question).tolist()

#     hits = client.search(
#         collection_name="book",
#         query_vector=vector,
#         limit=3
#     )

#     context = "\n".join([hit.payload["text"] for hit in hits])

#     return {
#         "answer": context
#     }
