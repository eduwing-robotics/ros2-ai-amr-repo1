// 로그인/회원가입 페이지 — 탭 전환으로 두 모드를 하나의 컴포넌트에서 처리
import { useState } from 'react';
import { login } from '../api/auth';
import { signup } from '../api/users';
import type { AuthUser } from '../types';

interface Props {
  onLogin: (user: AuthUser) => void;
}

type Mode = 'login' | 'signup';

export default function LoginPage({ onLogin }: Props) {
  const [mode, setMode] = useState<Mode>('login');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  // 탭 전환 시 입력값과 에러 초기화
  function switchMode(next: Mode) {
    setMode(next);
    setError('');
    setEmail('');
    setPassword('');
    setName('');
  }

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    setError('');
    setLoading(true);
    try {
      // 모드에 따라 로그인 또는 회원가입 API 호출
      // 회원가입 성공 시 서버가 AuthUser를 반환해 자동 로그인
      const user = mode === 'login'
        ? await login(email, password)
        : await signup(email, password, name);
      onLogin(user);
    } catch (err: unknown) {
      // 서버 에러 메시지 우선, 없으면 기본 문구 표시
      const msg = (err as { response?: { data?: { detail?: string } } })
        ?.response?.data?.detail;
      setError(msg ?? (mode === 'login' ? '이메일 또는 비밀번호가 올바르지 않습니다.' : '회원가입에 실패했습니다.'));
    } finally {
      setLoading(false);
    }
  }

  return (
    <div style={styles.wrap}>
      <div style={styles.card}>
        <div style={styles.logo}>🛒 S-Mart</div>
        <p style={styles.tag}>드라이브스루 스마트 마트</p>

        <div style={styles.tabs}>
          <button
            style={{ ...styles.tab, ...(mode === 'login' ? styles.tabActive : {}) }}
            onClick={() => switchMode('login')}
          >
            로그인
          </button>
          <button
            style={{ ...styles.tab, ...(mode === 'signup' ? styles.tabActive : {}) }}
            onClick={() => switchMode('signup')}
          >
            회원가입
          </button>
        </div>

        <form onSubmit={handleSubmit}>
          {/* 회원가입 모드일 때만 이름 필드 표시 */}
          {mode === 'signup' && (
            <>
              <label style={styles.label}>이름</label>
              <input
                style={styles.input}
                type="text"
                placeholder="홍길동"
                value={name}
                onChange={(e) => setName(e.target.value)}
                required
              />
            </>
          )}
          <label style={styles.label}>이메일</label>
          <input
            style={styles.input}
            type="text"
            placeholder="이메일"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
          <label style={styles.label}>비밀번호</label>
          <input
            style={styles.input}
            type="password"
            placeholder="비밀번호"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
          {error && <p style={styles.error}>{error}</p>}
          <button style={styles.btn} type="submit" disabled={loading}>
            {loading ? '처리 중...' : mode === 'login' ? '로그인' : '가입하기'}
          </button>
        </form>
      </div>
    </div>
  );
}

const styles: Record<string, React.CSSProperties> = {
  wrap: {
    minHeight: '100vh', background: '#f6f7f9',
    display: 'flex', alignItems: 'center', justifyContent: 'center',
  },
  card: {
    width: 360, background: '#fff',
    border: '1px solid #e5e7eb', borderRadius: 16, padding: 32,
  },
  logo: { fontSize: 24, fontWeight: 800, color: '#16a34a', textAlign: 'center', marginBottom: 6 },
  tag: { textAlign: 'center', color: '#6b7280', fontSize: 13, margin: '0 0 20px' },
  tabs: {
    display: 'flex', border: '1px solid #e5e7eb', borderRadius: 10,
    overflow: 'hidden', marginBottom: 20,
  },
  tab: {
    flex: 1, padding: '10px 0', border: 'none', background: '#fff',
    fontSize: 14, fontWeight: 600, color: '#6b7280', cursor: 'pointer',
  },
  tabActive: { background: '#16a34a', color: '#fff' },
  label: { display: 'block', fontSize: 13, fontWeight: 600, margin: '14px 0 6px' },
  input: {
    width: '100%', padding: '11px 13px', border: '1px solid #e5e7eb',
    borderRadius: 10, fontSize: 14, boxSizing: 'border-box',
  },
  error: { color: '#dc2626', fontSize: 13, margin: '10px 0 0' },
  btn: {
    width: '100%', padding: 12, background: '#16a34a', color: '#fff',
    border: 'none', borderRadius: 10, fontSize: 15, fontWeight: 700,
    cursor: 'pointer', marginTop: 20,
  },
};
