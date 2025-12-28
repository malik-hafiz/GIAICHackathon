import React, { useState } from 'react';

export default function Chatbot() {
  const [q, setQ] = useState('');
  const [a, setA] = useState('');

  async function ask() {
    const res = await fetch('http://127.0.0.1:8000/ask', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question: q })
    });
    const data = await res.json();
    setA(data.answer);
  }

  return (
    <div style={{border:'1px solid #ccc', padding:10}}>
      <h3>Ask the Book</h3>
      <input value={q} onChange={e => setQ(e.target.value)} />
      <button onClick={ask}>Ask</button>
      <pre>{a}</pre>
    </div>
  );
}
