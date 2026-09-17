import torch
import torch.nn as nn
import torch.nn.functional as F
import math

class HistEncoder(nn.Module):
    """编码历史轨迹序列"""
    def __init__(self, in_dim=4, hidden=128):
        super().__init__()
        # self.gru = nn.GRU(in_dim, hidden, batch_first=True)
        self.mlp = nn.Sequential(
            nn.Linear(120, 128),
            nn.ReLU(),
            nn.Linear(128, 128)
        )
    def forward(self, hist_seq):  # [B, T_h, D_h]
        # 16 * 30 * 128 h = 1 * 16 * 128
        # _, h = self.gru(hist_seq)
        # return h.squeeze(0)       # [B, hidden]
        B, T, D = hist_seq.shape
        x = hist_seq.view(B, -1)

        return self.mlp(x)

class CandEncoder(nn.Module):
    """编码每条候选轨迹（逐候选的序列编码）"""
    def __init__(self, in_dim=4, hidden=128):
        super().__init__()
        # self.gru = nn.GRU(in_dim, hidden, batch_first=True)
        self.mlp = nn.Sequential(
            nn.Linear(120, 128),
            nn.ReLU(),
            nn.Linear(128, 128)
        )
    def forward(self, cand_seq):  # [B, K = 6, T_f = 30 frame, D_c = 4 (x, y, dx, dy)]
        B, K, T, D = cand_seq.shape
        # x = cand_seq.view(B*K, T, D)
        # _, h = self.gru(x)
        # h = h.squeeze(0).view(B, K, -1)  # [B, K, hidden]
        # return h
        x = cand_seq.view(B*K, -1)
        h = self.mlp(x)
        h = h.squeeze(0).view(B, K, -1)  # [B, K, hidden]
        return h

class MDNHead(nn.Module):
    """
    feat_dim: Wi, distance to weight?
    输出:
      - mixture logits: [B, K]
      - residual delta: [B, K, T_f, 2] (对候选均值做细微调整)
      - log_sigma:      [B, K, 1] (每分量的各向同性方差，稳定性更好)
    """
    def __init__(self, hidden=128, T_future=30, feat_dim=2):
        super().__init__()
        out_res = T_future * 2
        self.mlp_res = nn.Sequential(
            nn.Linear(hidden*2 + feat_dim, 256),
            nn.ReLU(),
            nn.Linear(256, out_res)
        )
        self.mlp_logits = nn.Sequential(
            nn.Linear(hidden*2 + feat_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 1)
        )
        self.mlp_log_sigma = nn.Sequential(
            nn.Linear(hidden*2 + feat_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
        # self.mlp_res = MLP(hidden*2 + feat_dim, out_res, hidden) # 128
        # self.mlp_logits = MLP(hidden*2 + feat_dim, 1, hidden)
        # self.mlp_log_sigma = MLP(hidden*2 + feat_dim, 1, int(hidden/2))
        # self.mlp_res = nn.Sequential(
        #     MLP(hidden*2 + feat_dim, 256, 256),
        #     nn.Linear(256, out_res)
        # )
        # self.mlp_logits = nn.Sequential(
        #     MLP(hidden * 2 + feat_dim, 128, 256),
        #     nn.Linear(128, 1)
        # )
        # self.mlp_log_sigma = nn.Sequential(
        #     MLP(hidden * 2 + feat_dim, 64, 256),
        #     nn.Linear(64, 1)
        # )
        self.T_future = T_future

    def forward(self, h_hist, h_cand, cand_feat):
        B, K, H = h_cand.shape # H = 128
        hhist = h_hist.unsqueeze(1).expand(B, K, -1)         # [B,K,H]
        x = torch.cat([hhist, h_cand, cand_feat], dim=-1)    # [B,K,2H+feat] 16 6 260
        # residual
        res = self.mlp_res(x).view(B, K, self.T_future, 2)      # [B,K,T,2] 16 6 30 2
        # logits for mixture weights
        logits = self.mlp_logits(x).squeeze(-1)                  # [B,K]
        # log_sigma per component (broadcast along T and xy)
        log_sigma = self.mlp_log_sigma(x).squeeze(-1).unsqueeze(-1)  # [B,K,1]
        # log_sigma.fill_(0)
        return logits, res, log_sigma

class MDNModel(nn.Module):
    """把候选当作混合分量的均值（允许残差细化）
    x y dx dy
    """
    def __init__(self, in_dim_hist=4, in_dim_cand=4, hidden=128, T_future=30, feat_dim=24,device=torch.device("cpu")):
        super().__init__()
        self.enc_hist = HistEncoder(in_dim_hist, hidden)
        self.enc_cand = CandEncoder(in_dim_cand, hidden)
        self.head = MDNHead(hidden, T_future, feat_dim)
        self.T_future = T_future
        self.device = device
        self.fde_index = -1  # -1

    def forward(self, hist_seq, cand_seq, cand_feat, cand_prior_logits=None):
        """
        hist_seq: [B, T_h, D_h]
        cand_seq: [B, K, T_f, D_c]  (含候选的 x,y 及可能的 vx,vy 等)
        cand_feat: [B, K, F]        (手工或规则特征，如终点差、角度等)
        cand_prior_logits: [B, K]   (可选，外部 w_i 的 log 作为先验)
        """
        h_hist = self.enc_hist(hist_seq)                 # [B,H]
        h_cand = self.enc_cand(cand_seq)                 # [B,K,H]
        logits, residual, log_sigma = self.head(h_hist, h_cand, cand_feat)

        # if cand_prior_logits is not None:
        #     logits = logits + 0.5 * cand_prior_logits   # α=0.5 示例，可调
        # 分量权重
        pi = F.softmax(logits, dim=-1)                   # [B,K]
        # 分量均值轨迹 = 候选 + 残差
        mu = cand_seq[..., :2] + residual                # [B,K,T,2]
        # mu = cand_seq[..., :2]                # [B,K,T,2]
        # 标准差（各向同性），保证最小值
        log_sigma = torch.clamp(log_sigma, min=-1.0, max=0.5)  # σ ∈ [exp(-3), exp(3)]
        sigma = torch.exp(log_sigma)                     # [B,K,1]  -> broadcast

        # 2. 计算FDE（终点误差）作为额外约束
        # fde = torch.norm(residual[:, :, -1, :], dim=-1)  # [B,K]
        # # 3. 将FDE融入logits（FDE越小，概率越大）
        # fde_penalty = fde / 5.0  # 缩放系数
        # logits = logits - fde_penalty
        # pi = F.softmax(logits, dim=-1)                   # [B,K]

        # 4. 方差与混合系数相关：高? low pi对应小方差 diff big small var
        # sigma = base_sigma + (1-pi)*max_sigma
        # sigma_scale = 1.25 - (1.0 - pi.unsqueeze(-1)) / 2.0
        # sigma = sigma * sigma_scale

        sigma_shared = (pi.unsqueeze(-1) * sigma).sum(dim=1, keepdim=True).expand_as(sigma)

        return pi, mu, sigma_shared, logits

    def inference(self, hist_seq, cand_seq, cand_feat, cand_prior_logits=None):
        return self.forward(hist_seq, cand_seq, cand_feat, cand_prior_logits)

    # def gaussian_probability(self, target,  mu, sigma):
    #     """Returns the probability of `target` given MoG parameters `sigma` and `mu`.
    #     Arguments:
    #         sigma (BxGxO): The standard deviation of the Gaussians. B is the batch
    #             size, G is the number of Gaussians, and O is the number of
    #             dimensions per Gaussian.
    #         mu (BxGxO): The means of the Gaussians. B is the batch size, G is the
    #             number of Gaussians, and O is the number of dimensions per Gaussian.
    #         target (BxI): A batch of target. B is the batch size and I is the number of
    #             input dimensions.
    #     Returns:
    #         probabilities (BxG): The probability of each point in the probability
    #             of the distribution in the corresponding sigma/mu index.
    #     """
    #     target = target.unsqueeze(1).expand_as(sigma)
    #     ONEOVERSQRT2PI = 1.0 / math.sqrt(2 * math.pi)
    #     ret = ONEOVERSQRT2PI * torch.exp(-0.5 * ((target - mu) / sigma) ** 2) / sigma
    #     return torch.prod(ret, 2)
    #
    # def mdn_loss(self, pi, mu, sigma, Y, eps=1e-9):
    #     """Calculates the error, given the MoG parameters and the target
    #     The loss is the negative log likelihood of the data given the MoG
    #     parameters.
    #     """
    #     # t_pi = torch.exp(log_prob_comp)
    #     prob = pi * self.gaussian_probability(Y, mu, sigma)
    #     nll = -torch.log(torch.sum(prob, dim=1))
    #     return torch.mean(nll)

    def gaussian_log_prob(self, Y, mu, sigma):
        """
        计算每分量、每时刻的二维高斯对数似然（各向同性，x/y 同 σ）
        Y:   [B, T, 2] 16 30 2
        mu:  [B, K, T, 2]
        sigma: [B, K, 1] -> broadcast 到 [B,K,T,1]
        返回: log_prob_per_comp_time [B, K, T]
        """
        B, K, T, _ = mu.shape
        ty = Y[..., :2]
        diff = ty.unsqueeze(1) - mu                     # [B,K,T,2]
        var = (sigma ** 2).unsqueeze(-1)              # [B,K,1,1] -> [B,K,T,1] via broadcast

        # 对角且各向同性：log N(x|μ,σ^2 I) = - (||x-μ||^2)/(2σ^2) - d*log(σ) - d*log(√(2π))
        d = 2
        quad = (diff ** 2).sum(dim=-1) / (2.0 * var.squeeze(-1))      # [B,K,T]
        log_norm = d * torch.log(sigma) + d * 0.5 * torch.log(torch.tensor(2.0 * 3.141592653589793))
        log_prob = -(quad + log_norm)                                 # [B,K,T]
        # log_prob = -quad
        return log_prob

    def laplace_log_prob(self, Y, mu, sigma):
        """
        计算每分量、每时刻的二维高斯对数似然（各向同性，x/y 同 σ）
        Y:   [B, T, 2] 16 30 2
        mu:  [B, K, T, 2]
        sigma: [B, K, 1] -> broadcast 到 [B,K,T,1]
        返回: log_prob_per_comp_time [B, K, T]
        """
        B, K, T, _ = mu.shape
        ty = Y[..., :2]
        diff = ty.unsqueeze(1) - mu                     # [B,K,T,2]
        l1_norm = torch.abs(diff).sum(dim=-1)  # [B, K, T]
        d = 2  # 二维

        # l1_norm = torch.sqrt((diff ** 2).sum(dim=-1) + 1e-8)   # [B, K, T]
        # d = 1  # 二维

        log_prob = -d * torch.log(2 * sigma) - l1_norm / sigma

        # 计算对数似然
        # 由于 sigma: [B, K, 1] 可以与 l1_norm: [B, K, T] 广播
        # log_prob = -2 * torch.log(2 * sigma) - l2_dist / sigma
        return log_prob

    def mdn_nll(self, pi, mu, sigma, Y, eps=1e-9):
        """
        MDN 的负对数似然，稳定版 log-sum-exp。
        pi:   [B, K]
        mu:   [B, K, T, 2]
        sigma:[B, K, 1]
        Y:    [B, T, 2]
        返回: 标量损失
        """
        B, K = pi.shape
        # log_prob_t = self.gaussian_log_prob(Y, mu, sigma)       # [B,K,T]
        log_prob_t = self.laplace_log_prob(Y, mu, sigma)       # [B,K,T]
        # 时间独立假设: sum_t log p_t
        t = Y[:, self.fde_index, :2].unsqueeze(1) - mu[:, :, self.fde_index, :]
        fde = torch.linalg.norm(t, dim=-1)# [B,K]
        fde_normalized = torch.log(fde + 1.25)  # [B,K]
        # 用温度系数控制softmax尖锐程度 自适应权重：FDE越小权重越大
        temperature = 0.3
        weights = F.softmax(-fde_normalized / temperature, dim=-1) # [B,K]
        # weights = F.softmax(-fde.detach(), dim=-1)

        # log_prob_comp = log_prob_t.sum(dim=-1)             # [B,K]
        # log_prob_comp = log_prob_t.mean(dim=-1)             # [B,K]
        # log_prob_comp = log_prob_t[..., -1] + log_prob_t.max(dim=-1).values            # [B,K]
        # log_prob_comp = log_prob_t[..., -1] + 0.1 * log_prob_t.mean(dim=-1)            # [B,K]
        log_prob_comp = log_prob_t[..., self.fde_index]            # [B,K]
        # log_prob_comp = -fde_normalized / temperature           # [B,K]
        # 加上混合权重的 log
        log_pi = torch.log(pi + eps)                       # [B,K]
        comp_log = log_pi + log_prob_comp                  # [B,K]
        # log-sum-exp across K
        m = comp_log.max(dim=-1, keepdim=True).values      # [B,1]
        lse = m + torch.log(torch.exp(comp_log - m).sum(dim=-1, keepdim=True) + eps)  # [B,1]
        nll = -lse.mean()

        # m_label = log_prob_comp.max(dim=-1, keepdim=True).values      # [B,1]
        # lse_label = log_prob_comp - m_label - torch.log(torch.exp(log_prob_comp - m_label).sum(dim=-1, keepdim=True) + eps)
        # q_dist = torch.exp(lse_label)
        q_dist = F.softmax(log_prob_comp.detach(), dim=-1)  # 数据驱动的"后验" [B, K]
        # q_dist = F.softmax(log_prob_comp, dim=-1)  # 数据驱动的"后验" [B, K]
        # q_dist = F.softmax(weights+q_dist, dim=-1)
        kl_loss = F.kl_div(
            input=log_pi,  # 预测分布的对数
            target=weights,  # 目标分布
            reduction='batchmean',  # 平均
            log_target=False
        )

        # 获取硬标签：哪个分量后验概率最高 # 这相当于E步的硬分配
        pseudo_labels = weights.argmax(dim=-1)  # [B]
        # 注意：F.cross_entropy期望logits（未归一化的对数概率）
        ce_loss = F.nll_loss(log_pi, pseudo_labels, reduction='mean')

        # sigma_reg = (torch.mean(sigma ** 2) + torch.max(sigma ** 2)) * 0.01

        # 3. 组合损失
        # total_loss = nll + 2 * kl_loss + 0.1 * ce_loss# 0.05
        total_loss = nll + 2 * kl_loss# 0.05

        return total_loss

    def loss(self, pi, mu, sigma, logits, gt_future, cand_prior_logits=None):
        '''
        # hist_seq: [B, T_h, D_h]
          cand_seq: [B, K, T_f, D_c]
          cand_feat:[B, K, F]
          gt_future:[B, T_f, 2]
        :return:
        '''
        # 可选：终点强化（FDE 的软最小）
        # fde = torch.linalg.norm(mu[..., -1, :] - gt_future[..., -1, :2].unsqueeze(1), dim=-1)  # [B,K]
        # temp = 5.0
        # softmin_fde = -(torch.log((pi * torch.exp(-temp * fde)).sum(dim=-1) + 1e-12)).mean()

        top1 = pi.argmax(dim=-1)
        idx = torch.arange(pi.shape[0])
        fde = torch.linalg.norm(mu[idx, top1, -1, :] - gt_future[..., -1, :2], dim=-1)  # [B,K]
        temp = 5.0
        pi_max = pi[idx, top1]
        softmin_fde = -(torch.log((torch.exp(-temp * fde)).sum(dim=-1) + 1e-12)).mean()

        # 可选：与先验校准（若先验可信）
        if cand_prior_logits is not None:
            prior = F.softmax(cand_prior_logits, dim=-1)
            pred = F.softmax(logits, dim=-1)
            loss_cal = F.kl_div(pred.log(), prior, reduction='batchmean')
        else:
            loss_cal = torch.tensor(0.0)
        loss_nll = self.mdn_nll(pi, mu, sigma, gt_future)
        loss = loss_nll
        # loss = loss_nll + 0.03 * softmin_fde + 0.01 * loss_cal
        # loss = loss_nll + 0.01 * softmin_fde
        return loss
