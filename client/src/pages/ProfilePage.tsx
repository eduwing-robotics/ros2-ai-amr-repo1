// 프로필 페이지 — 이름 수정, 비밀번호 변경, 회원탈퇴
import { useEffect, useState } from 'react';
import { fetchProfile, updateProfile, deleteAccount, type Profile } from '../api/users';

interface Props {
  userId: number;
  onNameChange: (name: string) => void;   // 이름 변경 시 App.tsx 헤더 실시간 반영
  onDeleteAccount: () => void;             // 탈퇴 완료 시 App.tsx에서 로그아웃 처리
}

export default function ProfilePage({ userId, onNameChange, onDeleteAccount }: Props) {
  const [profile, setProfile] = useState<Profile | null>(null);
  const [name, setName] = useState('');
  const [currentPw, setCurrentPw] = useState('');
  const [newPw, setNewPw] = useState('');
  const [confirmPw, setConfirmPw] = useState('');
  const [infoMsg, setInfoMsg] = useState('');
  const [infoErr, setInfoErr] = useState('');
  const [pwMsg, setPwMsg] = useState('');
  const [pwErr, setPwErr] = useState('');
  const [saving, setSaving] = useState(false);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false);
  const [deleting, setDeleting] = useState(false);

  useEffect(() => {
    fetchProfile(userId).then((p) => {
      setProfile(p);
      setName(p.name);
    });
  }, [userId]);

  // 이름 변경 — 성공 시 헤더 이름도 즉시 업데이트
  async function handleNameSave(e: React.FormEvent) {
    e.preventDefault();
    setInfoMsg(''); setInfoErr('');
    setSaving(true);
    try {
      const updated = await updateProfile(userId, { name });
      setProfile(updated);
      onNameChange(updated.name);
      setInfoMsg('이름이 변경되었습니다.');
    } catch {
      setInfoErr('저장에 실패했습니다.');
    } finally {
      setSaving(false);
    }
  }

  // 비밀번호 변경 — 클라이언트 불일치 검증 후 API 호출
  async function handlePwSave(e: React.FormEvent) {
    e.preventDefault();
    setPwMsg(''); setPwErr('');
    // 1단계: 클라이언트 검증 (API 호출 없이 즉시 처리)
    if (newPw !== confirmPw) { setPwErr('새 비밀번호가 일치하지 않습니다.'); return; }
    setSaving(true);
    try {
      await updateProfile(userId, { current_password: currentPw, new_password: newPw });
      setCurrentPw(''); setNewPw(''); setConfirmPw('');
      setPwMsg('비밀번호가 변경되었습니다.');
    } catch (err: unknown) {
      // 2단계: 서버 검증 실패 시 API 에러 메시지 표시 (현재 비번 틀림 등)
      const msg = (err as { response?: { data?: { detail?: string } } })?.response?.data?.detail;
      setPwErr(msg ?? '비밀번호 변경에 실패했습니다.');
    } finally {
      setSaving(false);
    }
  }

  // 회원탈퇴 — 진행 중 주문 있으면 서버에서 409, alert으로 표시
  async function handleDelete() {
    setDeleting(true);
    try {
      await deleteAccount(userId);
      onDeleteAccount();
    } catch (err: unknown) {
      const msg = (err as { response?: { data?: { detail?: string } } })?.response?.data?.detail;
      alert(msg ?? '탈퇴에 실패했습니다.');
      setShowDeleteConfirm(false);
    } finally {
      setDeleting(false);
    }
  }

  if (!profile) return null;

  return (
    <div style={styles.wrap}>
      <h2 style={styles.h2}>내 정보</h2>

      {/* 기본 정보 */}
      <div style={styles.section}>
        <div style={styles.sectionTitle}>기본 정보</div>
        <div style={styles.infoRow}>
          <span style={styles.infoLabel}>이메일</span>
          <span style={styles.infoValue}>{profile.email}</span>
        </div>
        <form onSubmit={handleNameSave} style={styles.form}>
          <label style={styles.label}>이름</label>
          <div style={styles.row}>
            <input
              style={styles.input}
              value={name}
              onChange={(e) => setName(e.target.value)}
              required
            />
            <button style={styles.btnSave} type="submit" disabled={saving}>저장</button>
          </div>
          {infoMsg && <p style={styles.success}>{infoMsg}</p>}
          {infoErr && <p style={styles.error}>{infoErr}</p>}
        </form>
      </div>

      {/* 비밀번호 변경 */}
      <div style={styles.section}>
        <div style={styles.sectionTitle}>비밀번호 변경</div>
        <form onSubmit={handlePwSave} style={styles.form}>
          <label style={styles.label}>현재 비밀번호</label>
          <input style={styles.input} type="password" value={currentPw}
            onChange={(e) => setCurrentPw(e.target.value)} required />
          <label style={styles.label}>새 비밀번호</label>
          <input style={styles.input} type="password" value={newPw}
            onChange={(e) => setNewPw(e.target.value)} required />
          <label style={styles.label}>새 비밀번호 확인</label>
          <input style={styles.input} type="password" value={confirmPw}
            onChange={(e) => setConfirmPw(e.target.value)} required />
          {pwMsg && <p style={styles.success}>{pwMsg}</p>}
          {pwErr && <p style={styles.error}>{pwErr}</p>}
          <button style={styles.btnSave} type="submit" disabled={saving}>변경</button>
        </form>
      </div>

      {/* 회원 탈퇴 */}
      <div style={styles.section}>
        <div style={styles.sectionTitle}>회원 탈퇴</div>
        <p style={styles.warnText}>탈퇴 시 주문 내역이 모두 삭제되며 복구할 수 없습니다. 진행 중인 주문이 있으면 탈퇴할 수 없습니다.</p>
        {!showDeleteConfirm ? (
          <button style={styles.btnDelete} onClick={() => setShowDeleteConfirm(true)}>
            회원 탈퇴
          </button>
        ) : (
          // 2단계 확인 다이얼로그
          <div style={styles.confirmBox}>
            <p style={styles.confirmText}>정말 탈퇴하시겠습니까?</p>
            <div style={styles.confirmBtns}>
              <button style={styles.btnDeleteConfirm} onClick={handleDelete} disabled={deleting}>
                {deleting ? '처리 중...' : '탈퇴 확인'}
              </button>
              <button style={styles.btnCancelConfirm} onClick={() => setShowDeleteConfirm(false)}>
                취소
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

const styles: Record<string, React.CSSProperties> = {
  wrap: { maxWidth: 480, margin: '0 auto' },
  h2: { fontSize: 18, margin: '0 0 20px' },
  section: {
    background: '#fff', border: '1px solid #e5e7eb', borderRadius: 14,
    padding: 20, marginBottom: 16,
  },
  sectionTitle: { fontWeight: 700, fontSize: 15, marginBottom: 14, color: '#1f2937' },
  infoRow: { display: 'flex', alignItems: 'center', gap: 12, marginBottom: 4 },
  infoLabel: { fontSize: 13, color: '#6b7280', width: 60 },
  infoValue: { fontSize: 14, color: '#1f2937' },
  form: { display: 'flex', flexDirection: 'column', gap: 6 },
  label: { fontSize: 13, fontWeight: 600, marginTop: 10 },
  row: { display: 'flex', gap: 8 },
  input: {
    flex: 1, padding: '10px 12px', border: '1px solid #e5e7eb',
    borderRadius: 10, fontSize: 14, boxSizing: 'border-box',
  },
  btnSave: {
    padding: '10px 18px', background: '#16a34a', color: '#fff',
    border: 'none', borderRadius: 10, fontSize: 14, fontWeight: 700, cursor: 'pointer',
  },
  success: { color: '#16a34a', fontSize: 13, margin: '4px 0 0' },
  error: { color: '#dc2626', fontSize: 13, margin: '4px 0 0' },
  warnText: { fontSize: 13, color: '#6b7280', margin: '0 0 12px' },
  btnDelete: {
    padding: '10px 18px', background: '#fff', color: '#dc2626',
    border: '1px solid #dc2626', borderRadius: 10, fontSize: 14, fontWeight: 700, cursor: 'pointer',
  },
  confirmBox: { background: '#fee2e2', borderRadius: 10, padding: 14 },
  confirmText: { fontSize: 14, fontWeight: 600, color: '#dc2626', margin: '0 0 12px' },
  confirmBtns: { display: 'flex', gap: 8 },
  btnDeleteConfirm: {
    padding: '9px 16px', background: '#dc2626', color: '#fff',
    border: 'none', borderRadius: 10, fontSize: 14, fontWeight: 700, cursor: 'pointer',
  },
  btnCancelConfirm: {
    padding: '9px 16px', background: '#fff', color: '#6b7280',
    border: '1px solid #e5e7eb', borderRadius: 10, fontSize: 14, fontWeight: 700, cursor: 'pointer',
  },
};
