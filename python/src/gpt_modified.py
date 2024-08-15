import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader

class GPT(nn.Module):
    def __init__(self, vocab_size, embedding_dim, hidden_dim, num_blocks, num_heads, dropout):
        super(GPT, self).__init__()

        self.vocab_size = vocab_size
        self.embedding_dim = embedding_dim
        self.hidden_dim = hidden_dim
        self.num_blocks = num_blocks
        self.num_heads = num_heads
        self.dropout = dropout

        # Linear layer for encoding
        self.encoder = nn.Linear(vocab_size, embedding_dim)

        # Positional encoding
        self.positional_encoding = PositionalEncoding(embedding_dim)

        # Transformer blocks
        self.transformer_blocks = nn.ModuleList([
            TransformerBlock(embedding_dim, hidden_dim, num_heads, dropout)
            for _ in range(num_blocks)
        ])

        # Output linear layer
        self.fc = nn.Linear(embedding_dim, vocab_size)

    def forward(self, input_sequence):
        batch_size, sequence_length = input_sequence.size()

        # Encoding the input sequence
        encoded = self.encoder(input_sequence)  # shape: (batch_size, sequence_length, embedding_dim)

        # Positional encoding
        encoded = self.positional_encoding(encoded)

        # Transformer blocks
        for block in self.transformer_blocks:
            encoded = block(encoded)

        # Output linear layer
        output = self.fc(encoded)  # shape: (batch_size, sequence_length, vocab_size)

        return output


class PositionalEncoding(nn.Module):
    def __init__(self, embedding_dim, max_sequence_length=512):
        super(PositionalEncoding, self).__init__()

        self.embedding_dim = embedding_dim

        # Create positional encoding matrix
        pe = torch.zeros(max_sequence_length, embedding_dim)
        position = torch.arange(0, max_sequence_length, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, embedding_dim, 2).float() * (-torch.log(torch.tensor(10000.0)) / embedding_dim))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)

        # Register as a buffer (not a parameter) so it is saved along with the module state
        self.register_buffer('pe', pe)

    def forward(self, x):
        # Add positional encoding to the input
        x = x + self.pe[:x.size(1), :]
        return x


class TransformerBlock(nn.Module):
    def __init__(self, embedding_dim, hidden_dim, num_heads, dropout):
        super(TransformerBlock, self).__init__()

        self.attention = MultiHeadAttention(embedding_dim, num_heads, dropout)
        self.feedforward = FeedForward(embedding_dim, hidden_dim, dropout)
        self.layer_norm1 = nn.LayerNorm(embedding_dim)
        self.layer_norm2 = nn.LayerNorm(embedding_dim)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x):
        attended = self.attention(x)
        x = x + self.dropout(attended)
        x = self.layer_norm1(x)

        fed_forward = self.feedforward(x)
        x = x + self.dropout(fed_forward)
        x = self.layer_norm2(x)

        return x


class MultiHeadAttention(nn.Module):
    def __init__(self, embedding_dim, num_heads, dropout):
        super(MultiHeadAttention, self).__init__()

        # or use torch.nn.MultiheadAttention

        self.embedding_dim = embedding_dim
        self.num_heads = num_heads
        self.head_dim = embedding_dim // num_heads

        self.query = nn.Linear(embedding_dim, embedding_dim)
        self.key = nn.Linear(embedding_dim, embedding_dim)
        self.value = nn.Linear(embedding_dim, embedding_dim)

        self.fc = nn.Linear(embedding_dim, embedding_dim)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x):
        batch_size, sequence_length, embedding_dim = x.size()

        # Reshape input for multi-head attention
        q = self.query(x).view(batch_size, sequence_length, self.num_heads, self.head_dim).transpose(1, 2)
        k = self.key(x).view(batch_size, sequence_length, self.num_heads, self.head_dim).transpose(1, 2)
        v = self.value(x).view(batch_size, sequence_length, self.num_heads, self.head_dim).transpose(1, 2)

        # Calculate attention scores
        scores = torch.matmul(q, k.transpose(-2, -1)) / torch.sqrt(torch.tensor(self.head_dim, dtype=torch.float32))
        scores = F.softmax(scores, dim=-1)
        scores = self.dropout(scores)

        # Weighted sum of values
        weighted_sum = torch.matmul(scores, v)

        # Reshape and combine heads
        weighted_sum = weighted_sum.transpose(1, 2).contiguous().view(batch_size, sequence_length, embedding_dim)

        # Apply linear layer
        output = self.fc(weighted_sum)

        return output


class FeedForward(nn.Module):
    def __init__(self, embedding_dim, hidden_dim, dropout):
        super(FeedForward, self).__init__()

        self.embedding_dim = embedding_dim
        self.hidden_dim = hidden_dim

        self.linear1 = nn.Linear(embedding_dim, hidden_dim)
        self.activation = nn.ReLU()
        self.linear2 = nn.Linear(hidden_dim, embedding_dim)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x):
        x = self.linear1(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.linear2(x)

        return x

def training_example():
    # Define your dataset class here (providing access to your training data)

    # Define your training parameters
    vocab_size = 10000
    embedding_dim = 256
    hidden_dim = 512
    num_blocks = 12
    num_heads = 8
    dropout = 0.1
    batch_size = 32
    num_epochs = 10
    learning_rate = 0.001

    # Create instances of your model and dataset
    model = GPT(vocab_size, embedding_dim, hidden_dim, num_blocks, num_heads, dropout)
    dataset = YourDatasetClass()  # Replace YourDatasetClass with the class for your dataset

    # Create a DataLoader for training data
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    # Define the loss function and optimizer
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    # Set the model in training mode
    model.train()

    # Training loop
    for epoch in range(num_epochs):
        running_loss = 0.0

        for inputs, targets in dataloader:
            # Clear the gradients
            optimizer.zero_grad()

            # Forward pass
            outputs = model(inputs)

            # Compute the loss
            loss = criterion(outputs.view(-1, vocab_size), targets.view(-1))

            # Backward pass
            loss.backward()

            # Update the weights
            optimizer.step()

            # Accumulate the loss
            running_loss += loss.item()

        # Print the average loss for the epoch
        average_loss = running_loss / len(dataloader)
        print(f"Epoch {epoch+1}/{num_epochs} - Loss: {average_loss}")

    # Save the trained model
    torch.save(model.state_dict(), 'trained_model.pth')
