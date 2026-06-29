import { useEffect, useRef } from 'react';

// WebSocket 연결 훅 — 컴포넌트 마운트 시 서버와 연결 수립
// 서버 → 클라이언트 단방향 push 수신 (order_created, order_cancelled, location_updated)
// onMessage 콜백으로 메시지 처리 로직을 호출하는 쪽(App.tsx)에 위임
export function useWebSocket(onMessage: (data: unknown) => void) {
  // WebSocket 객체는 화면 렌더링과 무관 → useState 대신 useRef로 보관
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8000/ws');
    wsRef.current = ws;

    ws.onmessage = (e) => {
      try {
        onMessage(JSON.parse(e.data));
      } catch {
        // 잘못된 형식의 메시지는 무시
      }
    };

    // 컴포넌트 언마운트(로그아웃 등) 시 연결 종료
    return () => ws.close();
  }, []);
}
