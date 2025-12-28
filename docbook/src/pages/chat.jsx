import React, { useState } from "react";

export default function Chat() {
  const [q, setQ] = useState("");
  const [a, setA] = useState("");

  async function ask() {
    const res = await fetch("http://localhost:8000/ask", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ question: q })
    });
    const data = await res.json();
    setA(data.answer);
  }

  return (
    <div>
      <h1>Book AI Assistant</h1>
      <textarea onChange={e => setQ(e.target.value)} />
      <button onClick={ask}>Ask</button>
      <pre>{a}</pre>
    </div>
  );
}
