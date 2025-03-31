import SwiftUI

struct ContentView: View {
    @State private var qpSolution: [Double] = []

    var body: some View {
        VStack(spacing: 20) {
            Text("QP Solution:")
                .font(.headline)
            if qpSolution.isEmpty {
                Text("Calculating...")
            } else {
                Text(qpSolution.map { String(format: "%.4f", $0) }.joined(separator: ", "))
                    .font(.body)
            }
        }
        .padding()
        .onAppear {
            if let sol = solveExampleQP() as? [Double] {
                qpSolution = sol
            } else {
                qpSolution = []
            }
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
