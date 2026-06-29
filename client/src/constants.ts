// UI 표시용 상수 — DB에는 영어 값이 저장되므로 화면 출력 시 변환에 사용

// 상품명 → 이모지 매핑
export const EMOJI_MAP: Record<string, string> = {
  '사과':      '🍎',
  '배':        '🍐',
  '콜라':      '🥤',
  '생선':      '🐟',
  '아이스크림':'🍦',
  '냉동피자':  '🍕',
};

// OrderStatus 영어값 → 한글 변환
export const ORDER_STATUS_KO: Record<string, string> = {
  pending: '접수',
  processing: '출고중',
  awaiting_pickup: '수령 대기',
  delivered: '수령완료',
  cancelled: '취소됨',
};

// CancelReason 영어값 → 한글 변환
export const CANCEL_REASON_KO: Record<string, string> = {
  user: '고객 취소',
  no_pickup: '미수령 (시간 초과)',
  misdelivery: '오배송 정정',
};

// 주문 상태 스테퍼 4단계 (cancelled 제외 — 취소 시 스테퍼 미표시)
export const ORDER_STEPS: { status: string; label: string }[] = [
  { status: 'pending',         label: '접수' },
  { status: 'processing',      label: '출고중' },
  { status: 'awaiting_pickup', label: '수령 대기' },
  { status: 'delivered',       label: '수령완료' },
];
