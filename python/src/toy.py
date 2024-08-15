import torch
import numpy as np
import pickle
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
torch.cuda.empty_cache()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class NeuralNets(nn.Module):

    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(4, 30)
        #self.fc2 = nn.Linear(20,20)
        self.fc2 = nn.Linear(30, 2)

    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)

        #x = self.fc2(x)
        #x = F.relu(x)

        x = self.fc2(x)
        return x

def train_model(data, net, opt, ep):
    net.train()
    loss_criterion = nn.MSELoss()
    train_loss = 0
    num_batch = 0
    for i, sample in enumerate(data):
        inp, gt = sample[:,:4], sample[:,4:]
        num_batch += 1
        opt.zero_grad()
        pred = net(inp)
        loss = loss_criterion(pred, gt)
        train_loss += loss.item()
        opt.step()

    print('====> Epoch: {} Average Training Loss: {:.6f}'.format(
              ep+1, train_loss/num_batch)) 

def test_model(data, net):
    net.eval()
    test_loss = 0
    loss_criterion = nn.MSELoss()
    num_batch = 0  
    with torch.no_grad():
        for batch_idx, sample in enumerate(data):
            num_batch += 1
            inp, gt = sample[:,:4], sample[:,4:]

            output = net(inp)
            print(output[-1] - gt[-1])
            loss = loss_criterion(output, gt)
            test_loss += loss.item()
    test_loss /= num_batch


    print('Test set: Average Loss: {:.10f}\n'.format(test_loss))

def weights_init(m):
    classname = m.__class__.__name__
    if classname.find('Conv2d') != -1:
        torch.nn.init.xavier_normal_(m.weight.data)
        torch.nn.init.constant_(m.bias.data, 0.0)
    elif classname.find('Linear') != -1:
        torch.nn.init.xavier_normal_(m.weight.data)
        torch.nn.init.constant_(m.bias.data, 0.0)

def main():
    points = np.random.random_sample((1000, 4)) * 2
    points = np.round(points, decimals=2)
    gt = np.array([points[:,2] - points[:,0], points[:,3] - points[:,1]]).T
    points = np.hstack(( points, gt ))
    train, test = points[:800], points[:200]
    train = torch.from_numpy(train).float().to(device)
    test = torch.from_numpy(test).float().to(device)


    train_loader = torch.utils.data.DataLoader(train, batch_size=20, shuffle=True)
    test_loader = torch.utils.data.DataLoader(test, batch_size=20, shuffle=True)

    model = NeuralNets()
    model.to(device)
    model.apply(weights_init)

    optimizer = optim.Adam(model.parameters(), lr=0.001)
    #scheduler = optim.lr_scheduler.StepLR(optimizer, 100, gamma=0.1)
    for epoch in range(1001):
        train_model(train_loader, model, optimizer, epoch)
        #scheduler.step()
        test_model(test_loader, model)
        if epoch % 100 == 0:
            torch.save(model.state_dict(), 'data/toyproblem.pth')

if __name__=="__main__":
    main()
