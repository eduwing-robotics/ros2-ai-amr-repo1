// 앱 진입점 — 전역 상태 관리, 탭 라우팅, WebSocket 이벤트 분기
import { useState } from 'react';
import LoginPage from './pages/LoginPage';
import CatalogPage from './pages/CatalogPage';
import OrdersPage from './pages/OrdersPage';
import ProfilePage from './pages/ProfilePage';
import { useWebSocket } from './hooks/useWebSocket';
import type { AuthUser } from './types';

type Tab = 'catalog' | 'orders' | 'profile';

export default function App() {
  // user가 null이면 로그인 페이지 표시, 값이 있으면 메인 화면
  const [user, setUser] = useState<AuthUser | null>(null);
  const [tab, setTab] = useState<Tab>('catalog');

  // 숫자를 1씩 올려 하위 컴포넌트의 useEffect를 재실행시키는 트리거
  const [orderRefresh, setOrderRefresh] = useState(0);
  const [productRefresh, setProductRefresh] = useState(0);

  // WebSocket 이벤트 수신 → 해당 페이지 데이터 재조회 트리거
  useWebSocket((data) => {
    const msg = data as { event: string };

    // 주문 상태 변화 이벤트 — 모두 주문 목록 재조회로 처리
    // order_created / order_cancelled: FastAPI가 직접 broadcast
    // order_status_updated: Task Manager·Fleet Manager가 NOTIFY → FastAPI LISTEN → broadcast
    //   (processing / awaiting_pickup / delivered / cancelled(no_pickup))
    if (
      msg.event === 'order_created' ||
      msg.event === 'order_cancelled' ||
      msg.event === 'order_status_updated'
    ) {
      setOrderRefresh((n) => n + 1);
    }

    if (msg.event === 'location_updated') {
      // Task Manager가 reclaim task 완료 시 발행 → 재고 재조회
      setProductRefresh((n) => n + 1);
    }
  });

  if (!user) {
    return <LoginPage onLogin={setUser} />;
  }

  return (
    <div style={{ background: '#f6f7f9', minHeight: '100vh' }}>
      <header style={styles.header}>
        <div style={styles.bar}>
          <div style={styles.logo}>🛒 S-Mart <small style={styles.logoSmall}>드라이브스루</small></div>
          <div style={{ flex: 1 }} />
          <span style={styles.userName}>👤 <b>{user.name}</b>님</span>
          <button style={styles.logoutBtn} onClick={() => { setUser(null); setTab('catalog'); }}>
            로그아웃
          </button>
        </div>
        <nav style={styles.nav}>
          {(['catalog', 'orders', 'profile'] as Tab[]).map((t) => (
            <button
              key={t}
              style={{ ...styles.navBtn, ...(tab === t ? styles.navBtnActive : {}) }}
              onClick={() => setTab(t)}
            >
              {t === 'catalog' ? '상품' : t === 'orders' ? '내 주문' : '내 정보'}
            </button>
          ))}
        </nav>
      </header>

      {/* React Router 없이 tab 상태값으로 조건부 렌더링 */}
      <main style={styles.main}>
        {tab === 'catalog' && (
          <CatalogPage
            userId={user.user_id}
            refreshTrigger={productRefresh}
            onOrderCreated={() => setOrderRefresh((n) => n + 1)}
          />
        )}
        {tab === 'orders' && (
          <OrdersPage userId={user.user_id} refreshTrigger={orderRefresh} />
        )}
        {tab === 'profile' && (
          <ProfilePage
            userId={user.user_id}
            onNameChange={(name) => setUser({ ...user, name })}
            onDeleteAccount={() => setUser(null)}
          />
        )}
      </main>
    </div>
  );
}

const styles: Record<string, React.CSSProperties> = {
  header: { background: '#fff', borderBottom: '1px solid #e5e7eb', position: 'sticky', top: 0, zIndex: 10 },
  bar: { maxWidth: 980, margin: '0 auto', padding: '14px 20px', display: 'flex', alignItems: 'center', gap: 16 },
  logo: { fontSize: 20, fontWeight: 800, color: '#16a34a', display: 'flex', alignItems: 'center', gap: 8 },
  logoSmall: { fontWeight: 600, color: '#6b7280', fontSize: 12 },
  userName: { fontSize: 14, color: '#6b7280' },
  logoutBtn: { background: 'none', border: 'none', color: '#6b7280', cursor: 'pointer', fontSize: 13, textDecoration: 'underline' },
  nav: { maxWidth: 980, margin: '0 auto', padding: '0 20px', display: 'flex', gap: 4 },
  navBtn: {
    background: 'none', border: 'none', padding: '12px 16px',
    fontSize: 15, fontWeight: 600, color: '#6b7280', cursor: 'pointer',
    borderBottom: '2px solid transparent',
  },
  navBtnActive: { color: '#16a34a', borderBottomColor: '#16a34a' },
  main: { maxWidth: 980, margin: '0 auto', padding: '24px 20px 60px' },
};
