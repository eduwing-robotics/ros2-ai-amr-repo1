// axios 인스턴스 — 모든 API 함수가 이 인스턴스를 공유
// baseURL을 한 번만 설정해두면 각 함수에서 경로만 작성하면 됨
// 서버 주소 변경 시 여기 한 곳만 수정
import axios from 'axios';

const api = axios.create({
  baseURL: 'http://localhost:8000',
});

export default api;
